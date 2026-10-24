"""The training dataset folder: its files, its splits and the guard against held-out leakage.

The dataset lives outside the repo, in ~/vespr_dataset by default, because recorded data never
goes into git:

    manifest.csv          one row per extracted frame (FrameRow)
    clips.csv             one row per extracted clip (ClipRow), with the branch it shows and
                          the part of it that was kept
    splits.csv            one row per clip: clip, split (train, val, hold or test)
    images/<frame_id>.png left infrared image, 8-bit, lossless
    labels/<frame_id>.txt YOLO segmentation polygons; present only once labeled
    to_label/<batch>.zip  new, unlabeled frames packed for upload to the labeling tool
    runs/<run_name>/      one folder per training run, with its run.json

Splits are by clip, never by frame, because neighbouring frames are near-copies. Clips of the same
branch are near-copies too, so every clip of a branch gets the same split. Clips that are held
out (hold or test) must never have been used by a training run; check_splits enforces both.

Plain Python, NumPy and PyYAML only, so this runs anywhere, without a camera or a GPU.
"""

import csv
import hashlib
import json
import os
from dataclasses import astuple, dataclass, fields
from pathlib import Path

import numpy as np
import yaml

DOMAINS = ('indoor', 'outdoor')
SPLITS = ('train', 'val', 'hold', 'test')
HELD_OUT_SPLITS = ('hold', 'test')  # never trained on, tuned with or scored during development
CLASS_NAMES = {0: 'branch'}
DEFAULT_DATASET = Path('~/vespr_dataset')
SPLITS_HEADER = ('clip', 'split')  # the columns of splits.csv
# The columns clips.csv must have. group and keep came later, so a clips.csv without them still
# reads.
CLIPS_REQUIRED = ('clip', 'domain', 'size', 'md5')


class DatasetError(Exception):
    """Something in the dataset folder or an input file is wrong. The message says what."""


class LeakageError(DatasetError):
    """A held-out clip (hold or test) was used by a training run."""


@dataclass(frozen=True)
class FrameRow:
    """One extracted frame, one row of manifest.csv."""

    frame_id: str  # <clip>_<frame_index>, see frame_id()
    clip: str  # the clip's name, such as 0042_2026-10-23T14-02-11
    domain: str  # 'indoor' or 'outdoor'
    frame_index: int  # position of the frame in its clip, counting from 0
    timestamp: float  # seconds
    batch: str  # the to_label batch it was packed in, '' until packed


@dataclass(frozen=True)
class ClipRow:
    """One extracted clip, one row of clips.csv."""

    clip: str
    domain: str
    size: int  # bytes of the .bag
    md5: str  # of the .bag, lowercase hex
    group: str = ''  # the branch it shows, such as 'park/7', from its .json; '' for none
    keep: str = ''  # the seconds extracted, such as '0-12.5', from its .json; '' for all


def frame_id(clip, frame_index):
    """Return the name of one frame of a clip, with the index zero-padded to 6 digits."""
    return f'{clip}_{frame_index:06d}'


def file_md5(path):
    """Return a file's MD5 checksum as lowercase hex.

    The file is read one megabyte at a time, so a 1 GB clip does not need 1 GB of memory.
    """
    digest = hashlib.md5()
    with open(path, 'rb') as file:
        while True:
            block = file.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def parse_label(text):
    """Read a YOLO segmentation label file into one (N, 2) array of x, y per polygon.

    Each line is a class id followed by the polygon's corners as x y pairs, normalized to 0 to 1
    by the image width and height. An empty file means the frame was labeled and shows no
    branch. Blank lines and Windows line endings are accepted. Raises DatasetError on a bad line.
    """
    polygons = []
    for number, line in enumerate(text.splitlines(), start=1):
        parts = line.split()
        if not parts:
            continue
        where = f'label line {number} ({line.strip()!r})'
        if parts[0] != '0':
            raise DatasetError(f'{where} has class {parts[0]}, but the only class is 0 (branch)')
        try:
            values = [float(value) for value in parts[1:]]
        except ValueError:
            raise DatasetError(f'{where} has a coordinate that is not a number') from None
        if len(values) % 2 != 0:
            raise DatasetError(f'{where} has an odd number of coordinates')
        if len(values) < 6:
            raise DatasetError(f'{where} has fewer than 3 points')
        # Written as "all inside" so that a NaN, which fails every comparison, is rejected too.
        if not all(0 <= value <= 1 for value in values):
            raise DatasetError(f'{where} has a coordinate outside 0 to 1')
        polygons.append(np.array(values, dtype=float).reshape(-1, 2))
    return polygons


def _read_csv(path, columns):
    """Return a CSV file's rows as dicts keyed by its header, or [] if the file is missing.

    The CSV files may be edited by hand. 'utf-8-sig' skips the byte order mark (BOM) that Excel
    and some editors put at the start of a UTF-8 file. Raises DatasetError naming the file if
    the header (its first line) lacks one of columns, for example because it was deleted.
    """
    if not path.exists():
        return []
    with open(path, newline='', encoding='utf-8-sig') as file:
        reader = csv.DictReader(file)
        header = reader.fieldnames or []  # None for an empty file
        missing = [column for column in columns if column not in header]
        if missing:
            raise DatasetError(
                f'{path}: its first line must be the header {",".join(columns)}, but it lacks '
                f'{", ".join(missing)}')
        return list(reader)


def _read_rows(path, columns, make_row):
    """Return make_row(row) for each row of a CSV file, or [] if the file is missing.

    make_row turns one row, a dict keyed by column, into a FrameRow or ClipRow. Raises
    DatasetError naming the file and line if a number does not parse.
    """
    rows = []
    for line, row in enumerate(_read_csv(path, columns), start=2):  # line 1 is the header
        try:
            rows.append(make_row(row))
        except (ValueError, TypeError) as error:  # TypeError: a short row's missing cell is None
            raise DatasetError(f'{path} line {line} cannot be read: {error}') from None
    return rows


def _write_csv(path, header, rows):
    """Write a header and rows (tuples) to a CSV file, replacing it in one step.

    This is an atomic replace: the rows go to a temporary file first, and os.replace swaps it in
    for the old file in a single operation, so a crash never leaves a half-written CSV.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    with open(temporary, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(rows)
    os.replace(temporary, path)


def _header(row_class):
    """Return a dataclass's field names, which are also its CSV column names."""
    return tuple(field.name for field in fields(row_class))


def _frame_row(row):
    """Return one row of manifest.csv, a dict keyed by column, as a FrameRow."""
    return FrameRow(row['frame_id'], row['clip'], row['domain'], int(row['frame_index']),
                    float(row['timestamp']), row['batch'])


def _clip_row(row):
    """Return one row of clips.csv, a dict keyed by column, as a ClipRow."""
    return ClipRow(row['clip'], row['domain'], int(row['size']), row['md5'],
                   row.get('group') or '', row.get('keep') or '')


class Dataset:
    """The dataset folder: where each file lives, and reading and writing its CSV files."""

    def __init__(self, root):
        self.root = Path(root).expanduser()
        self.images_dir = self.root / 'images'
        self.labels_dir = self.root / 'labels'
        self.to_label_dir = self.root / 'to_label'
        self.runs_dir = self.root / 'runs'
        self._manifest_csv = self.root / 'manifest.csv'
        self._clips_csv = self.root / 'clips.csv'
        self._splits_csv = self.root / 'splits.csv'

    def read_frames(self):
        """Return every row of manifest.csv as a FrameRow, in file order."""
        return _read_rows(self._manifest_csv, _header(FrameRow), _frame_row)

    def write_frames(self, rows):
        """Replace manifest.csv with these FrameRows."""
        _write_csv(self._manifest_csv, _header(FrameRow), [astuple(row) for row in rows])

    def read_clips(self):
        """Return every row of clips.csv as a ClipRow, in file order."""
        return _read_rows(self._clips_csv, CLIPS_REQUIRED, _clip_row)

    def write_clips(self, rows):
        """Replace clips.csv with these ClipRows."""
        _write_csv(self._clips_csv, _header(ClipRow), [astuple(row) for row in rows])

    def read_splits(self):
        """Return splits.csv as {clip: split}.

        splits.csv may be edited by hand, so every row is checked: a split that is not one of
        SPLITS, a clip that is not in clips.csv, or a clip listed twice is an error, never
        silently ignored.
        """
        known_clips = {row.clip for row in self.read_clips()}
        splits = {}
        for row in _read_csv(self._splits_csv, SPLITS_HEADER):
            clip, split = row['clip'], row['split']
            if split not in SPLITS:
                raise DatasetError(
                    f'{self._splits_csv}: clip {clip} has split {split!r}, '
                    f'which is not one of {", ".join(SPLITS)}')
            if clip not in known_clips:
                raise DatasetError(f'{self._splits_csv}: clip {clip} is not in clips.csv')
            if clip in splits:
                raise DatasetError(f'{self._splits_csv}: clip {clip} is listed more than once')
            splits[clip] = split
        return splits

    def write_splits(self, splits):
        """Replace splits.csv with {clip: split}, sorted by clip so it is easy to edit by hand."""
        _write_csv(self._splits_csv, SPLITS_HEADER, sorted(splits.items()))

    def image_path(self, frame_id):
        """Return where a frame's image is stored."""
        return self.images_dir / f'{frame_id}.png'

    def label_path(self, frame_id):
        """Return where a frame's label is stored once it is labeled."""
        return self.labels_dir / f'{frame_id}.txt'

    def is_labeled(self, frame_id):
        """Return True if the frame has a label file. An empty one still counts: no branch."""
        return self.label_path(frame_id).exists()


def split_group(row):
    """Return what a clip shares its split with: its domain and branch, or the clip alone.

    A ClipRow with no group is a group of one. The domain is part of it, so branch 1 indoors and
    branch 1 outdoors are different branches.
    """
    return f'{row.domain}/{row.group}' if row.group else row.clip


def group_split(clip, splits, clips):
    """Return the split another clip of the same group already has, or None.

    splits is {clip: split} and clips is {clip: ClipRow}.
    """
    group = split_group(clips[clip])
    for other, split in sorted(splits.items()):
        if other != clip and other in clips and split_group(clips[other]) == group:
            return split
    return None


def assign_split(clip, splits, clips):
    """Return the split for a clip that has no split yet.

    splits is {clip: split} for the clips that already have one, and clips is {clip: ClipRow}.
    A clip joins the split of its branch's other clips, whatever it is, hold and test included.
    A new branch, or a clip with no branch, gets train or val: about 1 in 5 goes to val.
    """
    existing = group_split(clip, splits, clips)
    if existing is not None:
        return existing
    # Val needs at least one clip per domain, or that domain cannot be scored.
    domain = clips[clip].domain
    domain_has_val = any(split == 'val' and c in clips and clips[c].domain == domain
                         for c, split in splits.items())
    if not domain_has_val:
        return 'val'
    # Otherwise hash the group, so the answer never depends on the order clips arrive, and
    # every clip of a branch gets the same answer.
    group = split_group(clips[clip])
    return 'val' if int(hashlib.md5(group.encode()).hexdigest(), 16) % 5 == 0 else 'train'


def run_files(dataset, run):
    """Return a run's folder, its best weights and its image size.

    Raises DatasetError naming the run if it does not exist or has no best weights yet.
    """
    run_dir = dataset.runs_dir / run
    if not run_dir.is_dir():
        raise DatasetError(f'run {run} does not exist: there is no folder {run_dir}')
    weights = run_dir / 'weights' / 'best.pt'
    if not weights.exists():
        raise DatasetError(f'run {run} has no {weights}, so it has not finished training')
    try:
        image_size = json.loads((run_dir / 'run.json').read_text())['image_size']
    except (OSError, ValueError, KeyError) as error:
        raise DatasetError(f'run {run}: its run.json cannot be read ({error!r})') from None
    return run_dir, weights, image_size


def used_clips(dataset):
    """Return every clip that any training run used for train or val, read from its run.json."""
    used = set()
    for run_json in sorted(dataset.runs_dir.glob('*/run.json')):
        try:
            clips = json.loads(run_json.read_text())['clips']
            used.update(clips['train'])
            used.update(clips['val'])
        except (ValueError, KeyError, TypeError) as error:
            raise DatasetError(f'{run_json} cannot be read ({error!r})') from None
    return used


def check_splits(dataset):
    """Raise LeakageError if a clip marked hold or test, or its branch, was used by a run.

    This is the leakage guard. Once a run has trained or tuned on a clip, scoring that clip
    later would report an optimistic, invalid result, so it can never be held out again. The
    same goes for a branch, whose clips are near-copies: a held clip is leaked if a run used
    any clip of its branch, and DatasetError is raised if the clips of one branch have
    different splits.
    """
    splits = dataset.read_splits()
    clips = dataset.read_clips()
    used = used_clips(dataset)
    # A run that used one clip of a branch has seen the whole branch. A clip without a branch is
    # a group of one, so for it this is the clip itself.
    used_groups = {split_group(row) for row in clips if row.clip in used}
    leaked = sorted(row.clip for row in clips
                    if splits.get(row.clip) in HELD_OUT_SPLITS
                    and (row.clip in used or split_group(row) in used_groups))
    if leaked:
        raise LeakageError(
            'these clips are marked hold or test in splits.csv, but a training run already used '
            'them, or another clip of their branch, so they can no longer be held out: '
            f'{", ".join(leaked)}')

    by_group = {}  # {group: {clip: split}} for the clips that have a split
    for row in clips:
        if row.clip in splits:
            by_group.setdefault(split_group(row), {})[row.clip] = splits[row.clip]
    for group, members in sorted(by_group.items()):
        if len(set(members.values())) > 1:
            listed = ', '.join(f'{clip}: {split}' for clip, split in sorted(members.items()))
            raise DatasetError(
                f'the clips of branch {group} have different splits in splits.csv ({listed}). '
                'Clips of one branch are near-copies, so give them all the same split.')


def labeled_frames(dataset, split, domain=None):
    """Return the labeled frames of the clips in one split (and domain), in manifest order."""
    splits = dataset.read_splits()
    return [
        row for row in dataset.read_frames()
        if splits.get(row.clip) == split
        and (domain is None or row.domain == domain)
        and dataset.is_labeled(row.frame_id)
    ]


def write_data_yaml(dataset, out_dir, train, val):
    """Write the split lists and the Ultralytics data YAML for one run. Return the YAML's path.

    train.txt and val.txt hold one absolute image path per line. Ultralytics finds each label by
    swapping images/ for labels/ in the image path, so no files are copied per split.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, rows in (('train.txt', train), ('val.txt', val)):
        lines = [f'{dataset.image_path(row.frame_id).absolute()}\n' for row in rows]
        (out_dir / name).write_text(''.join(lines))

    data = {
        'path': str(out_dir.absolute()),
        'train': 'train.txt',  # relative to path
        'val': 'val.txt',
        'names': CLASS_NAMES,
    }
    yaml_path = out_dir / 'data.yaml'
    yaml_path.write_text(yaml.safe_dump(data, sort_keys=False))
    return yaml_path
