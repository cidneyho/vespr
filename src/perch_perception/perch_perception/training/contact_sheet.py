"""Draw a random sample of labeled frames, with their masks, into one grid image to review by eye.

    python -m perch_perception.training.contact_sheet

This is the label QA pass (T4.5). Frames are sampled from every labeled frame in the manifest,
whatever its split, hold clips included, because looking at labels is not scoring. Frames whose
label is empty (no branch) are sampled too and show with nothing drawn. The same seed always
gives the same sheet. It is written to contact_sheet.png in the dataset folder.
"""

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

from perch_perception.training.dataset import (
    DEFAULT_DATASET, Dataset, DatasetError, parse_label, sample_frames)

MASK_COLOR = (0, 255, 0)  # green, in OpenCV's blue, green, red order
MASK_OPACITY = 0.4
CAPTION_HEIGHT = 14  # pixels of black band under each thumbnail for its frame id


def draw_polygons(image, polygons, color):
    """Return a BGR copy of image with each polygon filled at 40% opacity and outlined.

    image is grayscale or BGR. Each polygon is an (N, 2) array of x, y normalized to 0 to 1 by
    the image width and height, as parse_label returns them. The input image is not changed.
    """
    drawn = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image.copy()
    height, width = drawn.shape[:2]
    # Scale normalized coordinates to pixels. fillPoly and polylines want int32 corners.
    corners = [np.round(polygon * [width, height]).astype(np.int32) for polygon in polygons]

    # Alpha blending: fill the polygons on a copy, then mix the copy with the original, 40% and
    # 60%, so the image stays visible through the mask.
    filled = drawn.copy()
    cv2.fillPoly(filled, corners, color)
    drawn = cv2.addWeighted(filled, MASK_OPACITY, drawn, 1 - MASK_OPACITY, 0)
    # Outline at full strength, so the mask's edge is easy to check.
    cv2.polylines(drawn, corners, isClosed=True, color=color, thickness=1)
    return drawn


def read_image(path):
    """Return the image at path as stored (8-bit grayscale here). Raises DatasetError if not."""
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise DatasetError(f'{path} cannot be read as an image')
    return image


def _tile(dataset, frame_id, thumb_width, thumb_height):
    """Return one frame shrunk to a thumbnail, its mask drawn on, its frame id written below."""
    image = read_image(dataset.image_path(frame_id))
    try:
        polygons = parse_label(dataset.label_path(frame_id).read_text())
    except DatasetError as error:
        raise DatasetError(f'the label for frame {frame_id}: {error}') from None

    # INTER_AREA averages the pixels it shrinks together, which is the usual choice for
    # shrinking. The mask is drawn after, so its outline stays one pixel wide on the thumbnail.
    thumbnail = cv2.resize(image, (thumb_width, thumb_height), interpolation=cv2.INTER_AREA)
    thumbnail = draw_polygons(thumbnail, polygons, MASK_COLOR)

    tile = np.zeros((thumb_height + CAPTION_HEIGHT, thumb_width, 3), dtype=np.uint8)
    tile[:thumb_height] = thumbnail
    cv2.putText(tile, frame_id, (2, thumb_height + CAPTION_HEIGHT - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1, cv2.LINE_AA)
    return tile


def make_contact_sheet(dataset, count=50, seed=0, columns=10, thumb_width=212):
    """Return a grid image (BGR) of `count` randomly chosen labeled frames, masks drawn on.

    The frames are drawn from every labeled frame in the manifest, whatever its split. A count
    larger than the number of labeled frames means all of them. The tiles are in manifest
    order, filling each row left to right, and cells after the last tile stay black. Every
    thumbnail is thumb_width wide, with the height kept in the first frame's proportions.
    Raises DatasetError if no frame is labeled yet.
    """
    labeled = [row for row in dataset.read_frames() if dataset.is_labeled(row.frame_id)]
    if not labeled:
        raise DatasetError(f'{dataset.root} has no labeled frames yet. Import a batch first.')

    # The same seed picks the same frames, and they stay in manifest order.
    frames = sample_frames(labeled, count, seed)

    first = read_image(dataset.image_path(frames[0].frame_id))
    thumb_height = round(first.shape[0] * thumb_width / first.shape[1])
    tile_height = thumb_height + CAPTION_HEIGHT

    rows = math.ceil(len(frames) / columns)  # rounded up: a part-filled last row still counts
    sheet = np.zeros((rows * tile_height, columns * thumb_width, 3), dtype=np.uint8)
    for number, row in enumerate(frames):
        # Grid position: the row is the quotient, the column the remainder.
        top = (number // columns) * tile_height
        left = (number % columns) * thumb_width
        sheet[top:top + tile_height, left:left + thumb_width] = _tile(
            dataset, row.frame_id, thumb_width, thumb_height)
    return sheet


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--dataset', type=Path, default=DEFAULT_DATASET,
                        help=f'dataset folder (default {DEFAULT_DATASET})')
    parser.add_argument('--count', type=int, default=50,
                        help='how many labeled frames to draw (default 50)')
    parser.add_argument('--seed', type=int, default=0,
                        help='random seed; the same seed gives the same sheet (default 0)')
    parser.add_argument('--out', type=Path, default=None,
                        help='where to write the PNG (default <dataset>/contact_sheet.png)')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    dataset = Dataset(args.dataset)
    out = args.out.expanduser() if args.out else dataset.root / 'contact_sheet.png'
    try:
        sheet = make_contact_sheet(dataset, count=args.count, seed=args.seed)
        if not cv2.imwrite(str(out), sheet):
            raise DatasetError(f'{out} cannot be written')
    except DatasetError as error:
        print(f'error: {error}', file=sys.stderr)
        return 1

    print(f'wrote {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
