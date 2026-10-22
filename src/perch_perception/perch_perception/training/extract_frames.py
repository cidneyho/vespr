"""Extract frames from recorded .bag clips and pack the new ones into batches for labeling.

    python -m perch_perception.training.extract_frames --domain outdoor ~/vespr_recordings/outdoor

Every .bag in the folder whose clip is not in the dataset yet is replayed, and every Nth frame
(default 10, about 3 frames per second at 30 fps) is saved as a lossless 8-bit PNG of the left
infrared image. Clips already extracted are skipped, so the same folder can be run again as more
clips arrive. At the end, the frames not yet in a batch are zipped into to_label/batch_<nnn>.zip,
ready to upload to the labeling tool.

pyrealsense2 is imported only when a real .bag is opened, so this module imports without it.
"""

import argparse
import sys
import zipfile
from dataclasses import dataclass, replace
from pathlib import Path

import cv2

from perch_perception.training.dataset import (
    DEFAULT_DATASET,
    DOMAINS,
    ClipRow,
    Dataset,
    DatasetError,
    FrameRow,
    check_splits,
    file_md5,
    frame_id,
)


@dataclass
class ExtractSummary:
    """What one extract_folder run did."""

    extracted: list  # clip names extracted this run
    skipped: list  # clip names already in the dataset
    frames: int  # frames written this run
    batches: list  # paths of the batch zips written this run


def open_bag_source(path):
    """Open a .bag for replay. The default open_bag of extract_folder."""
    # Imported here, not at the top, so this module works on a machine without pyrealsense2.
    from perch_perception.realsense_source import BagSource

    return BagSource(path)


def extract_clip(dataset, frames, clip, domain, every):
    """Save every Nth frame of a clip as a PNG and return their FrameRows, not yet batched.

    Frames are counted from 0 in the order they arrive, and a frame is kept when its index is a
    multiple of every, so the first frame is always kept. Nothing is written to the CSV files.
    """
    dataset.images_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for index, frame in enumerate(frames):
        if index % every != 0:
            continue
        name = frame_id(clip, index)
        path = dataset.image_path(name)
        # PNG is lossless, and a 2-D uint8 image is written as 8-bit grayscale.
        if not cv2.imwrite(str(path), frame.image):
            raise DatasetError(f'could not write {path}')
        rows.append(FrameRow(name, clip, domain, index, frame.timestamp, ''))
    return rows


def _batch_number(batch):
    """Return the number in a batch name, for example 3 for 'batch_003'."""
    return int(batch.removeprefix('batch_'))


def pack_batches(dataset, batch_size=100):
    """Zip the frames that are in no batch yet into to_label/batch_<nnn>.zip. Return the paths.

    Frames are packed in manifest order, at most batch_size per zip, with the PNGs at the top of
    the zip and no folders. Numbering carries on from the highest batch in the manifest. Each
    packed frame's batch column is then set to its batch name.
    """
    rows = dataset.read_frames()
    unbatched = [index for index, row in enumerate(rows) if row.batch == '']
    numbers = [_batch_number(row.batch) for row in rows if row.batch != '']
    next_number = max(numbers, default=0) + 1

    dataset.to_label_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    # Take the unbatched frames batch_size at a time; the last batch holds what is left.
    for start in range(0, len(unbatched), batch_size):
        batch = f'batch_{next_number:03d}'
        next_number += 1
        path = dataset.to_label_dir / f'{batch}.zip'
        with zipfile.ZipFile(path, 'w') as archive:
            for index in unbatched[start:start + batch_size]:
                image = dataset.image_path(rows[index].frame_id)
                archive.write(image, arcname=image.name)
                rows[index] = replace(rows[index], batch=batch)
        paths.append(path)

    # The manifest is written once, after every zip exists, so a crash while zipping leaves
    # the frames unbatched and the next run packs them again.
    dataset.write_frames(rows)
    return paths


def extract_folder(dataset, folder, domain, every=10, hold=(), batch_size=100, open_bag=None):
    """Extract every new .bag clip in a folder, then pack the new frames into batches.

    The clip name is the .bag file's name without .bag. A clip already in clips.csv with the
    same MD5 is skipped; one with the same name but a different MD5 is refused. Clips named in
    hold are written to splits.csv as hold; each must be a new clip in the folder.

    Every check runs before anything is extracted. Then each clip's rows are written only after
    the clip has been read to the end, so a clip that fails partway leaves no rows and the next
    run tries it again. open_bag(path) returns a FrameSource to use in a with block.
    """
    if open_bag is None:
        open_bag = open_bag_source
    folder = Path(folder).expanduser()
    hold = list(hold)  # read more than once below
    if domain not in DOMAINS:
        raise DatasetError(f'domain {domain!r} is not one of {", ".join(DOMAINS)}')
    if every < 1 or batch_size < 1:
        raise DatasetError('every and batch_size must be at least 1')
    if not folder.is_dir():
        raise DatasetError(f'{folder} is not a folder')
    check_splits(dataset)

    # Sort out which clips are new and which were extracted before.
    known = {row.clip: row for row in dataset.read_clips()}
    new_clips = []  # (clip, path, ClipRow) for each clip to extract
    skipped = []
    for path in sorted(folder.glob('*.bag')):
        clip = path.stem
        md5 = file_md5(path)
        if clip not in known:
            new_clips.append((clip, path, ClipRow(clip, domain, path.stat().st_size, md5)))
        elif known[clip].md5 == md5:
            skipped.append(clip)
        else:
            raise DatasetError(
                f'{path.name} has the name of clip {clip}, which was already extracted, but '
                'different content. Rename the file if it is a different recording.')

    new_names = [clip for clip, _, _ in new_clips]
    for clip in hold:
        if clip in skipped:
            raise DatasetError(
                f'clip {clip} was already extracted, so --hold cannot mark it. '
                'Add or change its row in splits.csv by hand instead.')
        if clip not in new_names:
            raise DatasetError(f'--hold names clip {clip}, but there is no {clip}.bag in {folder}')

    frame_count = 0
    for clip, path, clip_row in new_clips:
        print(f'extracting {path.name}')
        try:
            with open_bag(path) as source:
                rows = extract_clip(dataset, source, clip, domain, every)
        except Exception as error:
            raise DatasetError(f'{path.name} could not be read to the end: {error}') from error

        # The clip has been read to the end, so its rows are written now: frames, then the clip,
        # then its hold split, because splits.csv may only name clips that are in clips.csv.
        # Any frames of this clip already in the manifest are dropped first: a crash between
        # writing the frames and the clip leaves them behind, and a retry must not add them twice.
        kept = [row for row in dataset.read_frames() if row.clip != clip]
        dataset.write_frames(kept + rows)
        dataset.write_clips(dataset.read_clips() + [clip_row])
        if clip in hold:
            splits = dataset.read_splits()
            splits[clip] = 'hold'
            dataset.write_splits(splits)
        frame_count += len(rows)

    batches = pack_batches(dataset, batch_size)
    return ExtractSummary(new_names, skipped, frame_count, batches)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('folder', type=Path, help='folder of recorded .bag clips')
    parser.add_argument('--domain', required=True, choices=DOMAINS,
                        help='where every clip in the folder was recorded')
    parser.add_argument('--dataset', type=Path, default=DEFAULT_DATASET,
                        help=f'dataset folder (default {DEFAULT_DATASET})')
    parser.add_argument('--every', type=int, default=10,
                        help='keep every Nth frame (default 10)')
    parser.add_argument('--hold', action='append', default=[], metavar='CLIP',
                        help='mark this new clip hold in splits.csv; repeat for more clips')
    parser.add_argument('--batch-size', type=int, default=100,
                        help='most frames per labeling batch (default 100)')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    dataset = Dataset(args.dataset)
    try:
        summary = extract_folder(dataset, args.folder, args.domain, args.every, args.hold,
                                 args.batch_size)
    except DatasetError as error:
        print(f'error: {error}', file=sys.stderr)
        return 1

    print(f'extracted {len(summary.extracted)} clips, {summary.frames} frames: '
          f'{", ".join(summary.extracted) or "none"}')
    print(f'skipped {len(summary.skipped)} clips already extracted')
    for path in summary.batches:
        print(f'to label: {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
