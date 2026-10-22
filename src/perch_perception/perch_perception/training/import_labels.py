"""Import a labeling tool's export of one finished batch into the dataset's labels folder.

    python -m perch_perception.training.import_labels ~/Downloads/batch_001_export.zip

The export is the tool's YOLO segmentation export (CVAT's "Ultralytics YOLO Segmentation" or
Roboflow's "YOLOv8/YOLO11 segmentation"), as a zip or an unzipped folder. Each label is matched
to its frame by file name and copied to labels/<frame_id>.txt. Clips that now have labels and no
split yet are given train or val in splits.csv.

The import is refused, changing nothing, unless the export covers exactly the frames of one
batch, every label matches a frame in the manifest, and every polygon is well formed.
"""

import argparse
import re
import sys
import zipfile
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

from perch_perception.training.dataset import (
    DEFAULT_DATASET,
    Dataset,
    DatasetError,
    assign_split,
    check_splits,
    parse_label,
)


@dataclass
class ImportSummary:
    """What one import_batch run did."""

    batch: str  # the batch the export covered, for example 'batch_001'
    labels: int  # label files imported
    empty: int  # of those, how many have no polygon: the frame shows no branch
    replaced: int  # of those, how many frames already had a label, now replaced
    new_splits: dict  # {clip: 'train' or 'val'} for clips given a split by this import


def original_stem(name):
    """Return a label file's stem as the frame id, undoing Roboflow's renaming.

    Roboflow renames an image a_000010.png to a_000010_png.rf.<hash>, so the original stem is
    what comes before _png.rf. CVAT keeps the name, which then comes back unchanged.
    """
    # A regular expression anchored at the end ($): '_png', '_jpg' or '_jpeg', then '.rf.' and
    # a hex hash. Only that exact suffix is cut off.
    return re.sub(r'_(png|jpg|jpeg)\.rf\.[0-9a-f]+$', '', name)


def _is_label_file(name):
    """Return True for a .txt file inside a folder named labels, outside macOS metadata.

    name is a path inside the export, with / between folders. Exports also hold data.yaml,
    train.txt, images and labels.cache files, and a zip made on a Mac holds a __MACOSX folder
    of ._<name> copies; none of those are labels.
    """
    parts = PurePosixPath(name).parts
    folders = parts[:-1]
    return name.endswith('.txt') and 'labels' in folders and '__MACOSX' not in folders


def _label_files(path):
    """Return (name inside the export, text) for each label file in a zip or a folder.

    The text is decoded as 'utf-8-sig', which skips the byte order mark (BOM) that some
    editors put at the start of a UTF-8 file, so a hand-fixed label still reads as class 0.
    """
    if path.is_dir():
        files = []
        for file in sorted(path.rglob('*.txt')):
            name = file.relative_to(path).as_posix()
            if _is_label_file(name):
                files.append((name, file.read_text(encoding='utf-8-sig')))
        return files
    if zipfile.is_zipfile(path):
        with zipfile.ZipFile(path) as archive:
            return [(name, archive.read(name).decode('utf-8-sig'))
                    for name in sorted(archive.namelist()) if _is_label_file(name)]
    raise DatasetError(f'{path} is not a zip file or a folder')


def read_export(path):
    """Return {frame_id: label text} for every label in an export, a zip or a folder.

    Only .txt files in a folder named labels, at any depth, are read. Raises DatasetError if
    two files in the export are labels for the same frame.
    """
    path = Path(path).expanduser()
    labels = {}
    sources = {}  # frame_id: name inside the export, to name both files of a duplicate
    for name, text in _label_files(path):
        frame = original_stem(PurePosixPath(name).stem)
        if frame in labels:
            raise DatasetError(
                f'{path}: {sources[frame]} and {name} are both labels for frame {frame}')
        labels[frame] = text
        sources[frame] = name
    return labels


def _some(names, limit=5):
    """Return names as one comma-separated string, cut short after limit names."""
    shown = ', '.join(names[:limit])
    if len(names) > limit:
        shown += f' and {len(names) - limit} more'
    return shown


def _clean_label(text):
    """Return label text with one polygon per line, single spaces and plain newlines.

    Windows line endings, trailing spaces and blank lines are dropped, so every tool that reads
    the file later sees the same lines parse_label checked. The numbers are kept as written.
    """
    lines = [' '.join(line.split()) for line in text.splitlines() if line.strip()]
    return ''.join(f'{line}\n' for line in lines)


def import_batch(dataset, export):
    """Import an export of one finished batch into labels/ and give new clips a split.

    Every check runs before anything is written: the export must hold labels for exactly the
    frames of one batch, and every polygon must be well formed. A frame that was already
    labeled gets its label replaced. Raises DatasetError, changing nothing, if a check fails.
    """
    check_splits(dataset)
    labels = read_export(export)
    if not labels:
        raise DatasetError(
            f'{export} has no .txt files in a labels folder. '
            'Export the batch in a YOLO segmentation format.')

    # Every label must belong to a frame that was packed into a batch.
    frames = dataset.read_frames()
    by_id = {row.frame_id: row for row in frames}
    unknown = sorted(name for name in labels if name not in by_id)
    if unknown:
        raise DatasetError(f'{len(unknown)} labels match no frame in manifest.csv: '
                           f'{_some(unknown)}')
    unpacked = sorted(name for name in labels if by_id[name].batch == '')
    if unpacked:
        raise DatasetError(f'{len(unpacked)} labels are for frames that were never packed into '
                           f'a batch: {_some(unpacked)}')

    # The export must cover one batch, all of it, because a tool may export an empty label for
    # a frame nobody looked at. Only a finished batch tells "no branch" from "not labeled".
    batches = sorted({by_id[name].batch for name in labels})
    if len(batches) > 1:
        raise DatasetError(f'the export holds labels from more than one batch: '
                           f'{", ".join(batches)}. Import one batch at a time.')
    batch = batches[0]
    batch_frames = [row.frame_id for row in frames if row.batch == batch]
    missing = [name for name in batch_frames if name not in labels]
    if missing:
        raise DatasetError(
            f'the export is missing {len(missing)} of {len(batch_frames)} frames of {batch}: '
            f'{_some(missing)}. Finish the whole batch in the labeling tool, marking frames '
            'with no branch, then export it again.')

    empty = 0
    for name in sorted(labels):
        try:
            polygons = parse_label(labels[name])
        except DatasetError as error:
            raise DatasetError(f'the label for frame {name}: {error}') from None
        if not polygons:
            empty += 1

    # Work out the new splits before writing anything, because a clip without a clips.csv row
    # cannot be given one: splits.csv may only name clips that are in clips.csv. A clip is
    # labeled once any of its frames has a label file, from this import or an earlier one.
    # Clips are taken in manifest order, so which clip is a domain's first val clip does not
    # depend on dict order.
    splits = dataset.read_splits()
    clip_domains = {row.clip: row.domain for row in dataset.read_clips()}
    new_splits = {}
    for row in frames:
        labeled = row.frame_id in labels or dataset.is_labeled(row.frame_id)
        if row.clip in splits or not labeled:
            continue
        if row.clip not in clip_domains:
            raise DatasetError(
                f'clip {row.clip} is not in clips.csv, though manifest.csv has frames of it, '
                'probably because its extraction stopped partway. Remove its rows from '
                'manifest.csv and extract it again.')
        split = assign_split(row.clip, clip_domains[row.clip], splits, clip_domains)
        splits[row.clip] = split
        new_splits[row.clip] = split

    # Everything checked, so now the labels and splits are written.
    dataset.labels_dir.mkdir(parents=True, exist_ok=True)
    replaced = 0
    for name in batch_frames:
        if dataset.is_labeled(name):
            replaced += 1
        dataset.label_path(name).write_text(_clean_label(labels[name]))
    dataset.write_splits(splits)
    check_splits(dataset)

    return ImportSummary(batch, len(labels), empty, replaced, new_splits)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export', type=Path,
                        help="the labeling tool's YOLO segmentation export, a zip or a folder")
    parser.add_argument('--dataset', type=Path, default=DEFAULT_DATASET,
                        help=f'dataset folder (default {DEFAULT_DATASET})')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    dataset = Dataset(args.dataset)
    try:
        summary = import_batch(dataset, args.export.expanduser())
    except DatasetError as error:
        print(f'error: {error}', file=sys.stderr)
        return 1

    print(f'imported {summary.labels} labels of {summary.batch}')
    print(f'replaced {summary.replaced} labels that were imported before')
    print(f'{summary.empty} of {summary.labels} frames came back with no branch')
    for clip, split in summary.new_splits.items():
        print(f'new clip {clip}: {split}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
