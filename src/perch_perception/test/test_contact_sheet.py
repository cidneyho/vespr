"""Tests for training/contact_sheet.py, which draws labeled frames into one grid for review.

Each dataset test builds a small dataset folder under tmp_path: frames in manifest.csv, each with
a small gray image whose brightness differs per frame, so two sheets of different frames differ.
"""

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from perch_perception.training.contact_sheet import (  # noqa: E402
    CAPTION_HEIGHT, draw_polygons, main, make_contact_sheet)
from perch_perception.training.dataset import (  # noqa: E402
    ClipRow, Dataset, DatasetError, FrameRow, frame_id)

CLIP = '2026-10-19_14-03-22'
HELD_CLIP = '2026-10-19_14-05-10'
IMAGE_WIDTH = 80
IMAGE_HEIGHT = 40
SQUARE = '0 0.25 0.25 0.75 0.25 0.75 0.75 0.25 0.75\n'


def make_dataset(root, labeled=12, unlabeled=0):
    """A dataset with frames split across a train clip and a hold clip.

    The first `labeled` frames have a label file, every second one empty (no branch), and the
    next `unlabeled` frames have none. Every frame has an 80x40 gray image.
    """
    dataset = Dataset(root)
    dataset.write_clips([ClipRow(CLIP, 'indoor', 100, 'f' * 32),
                         ClipRow(HELD_CLIP, 'indoor', 100, 'e' * 32)])
    dataset.write_splits({CLIP: 'train', HELD_CLIP: 'hold'})
    dataset.images_dir.mkdir(parents=True)
    dataset.labels_dir.mkdir(parents=True)
    rows = []
    for number in range(labeled + unlabeled):
        clip = CLIP if number % 2 else HELD_CLIP
        row = FrameRow(frame_id(clip, number), clip, 'indoor', number, number / 30, 'batch_001')
        rows.append(row)
        image = np.full((IMAGE_HEIGHT, IMAGE_WIDTH), 10 * number, dtype=np.uint8)
        cv2.imwrite(str(dataset.image_path(row.frame_id)), image)
        if number < labeled:
            dataset.label_path(row.frame_id).write_text(SQUARE if number % 2 else '')
    dataset.write_frames(rows)
    return dataset


def test_a_polygon_is_drawn_over_the_image():
    image = np.zeros((20, 20), dtype=np.uint8)
    square = np.array([[0.25, 0.25], [0.75, 0.25], [0.75, 0.75], [0.25, 0.75]])

    drawn = draw_polygons(image, [square], (0, 255, 0))

    assert drawn.shape == (20, 20, 3)
    assert drawn[10, 10].tolist() == [0, 102, 0]  # 40% of 255, blended over black
    assert drawn[0, 0].tolist() == [0, 0, 0]
    assert image.max() == 0  # the input is not changed


def test_the_same_seed_picks_the_same_frames(tmp_path):
    dataset = make_dataset(tmp_path / 'dataset')

    first = make_contact_sheet(dataset, count=3, seed=7, columns=3, thumb_width=40)
    again = make_contact_sheet(dataset, count=3, seed=7, columns=3, thumb_width=40)
    other = make_contact_sheet(dataset, count=3, seed=8, columns=3, thumb_width=40)

    assert np.array_equal(first, again)
    assert not np.array_equal(first, other)


def test_the_sheet_has_one_tile_per_sampled_frame(tmp_path):
    # 12 labeled frames, half of them negatives and half from a hold clip, plus 4 unlabeled
    # frames that must be left out. 12 tiles in 5 columns make 3 rows; 16 would make 4.
    dataset = make_dataset(tmp_path / 'dataset', labeled=12, unlabeled=4)

    sheet = make_contact_sheet(dataset, count=50, columns=5, thumb_width=40)

    tile_height = 20 + CAPTION_HEIGHT  # 80x40 images shrink to 40x20
    assert sheet.shape == (3 * tile_height, 5 * 40, 3)
    # The last row holds 2 tiles, and the 3 cells after them stay black.
    assert sheet[2 * tile_height:, 2 * 40:].max() == 0


def test_no_labeled_frames_is_an_error(tmp_path):
    dataset = make_dataset(tmp_path / 'dataset', labeled=0, unlabeled=3)

    with pytest.raises(DatasetError, match='no labeled frames'):
        make_contact_sheet(dataset)


def test_the_command_writes_the_sheet_into_the_dataset_folder(tmp_path, capsys):
    dataset = make_dataset(tmp_path / 'dataset')

    status = main(['--dataset', str(dataset.root), '--count', '4'])

    assert status == 0
    sheet = cv2.imread(str(dataset.root / 'contact_sheet.png'))
    assert sheet is not None
    assert str(dataset.root / 'contact_sheet.png') in capsys.readouterr().out
