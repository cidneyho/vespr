"""A tiny generated training dataset, for tests that train, evaluate or export a model.

Five clips of three 96x96 frames each. Two frames per clip are a mid-gray image with a bright bar
(the "branch") drawn at a different angle, labeled with the bar's rotated-rectangle polygon. The
third is plain gray with an empty label: a frame with no branch. This checks the plumbing, not
accuracy.
"""

import cv2
import numpy as np

from perch_perception.training.dataset import ClipRow, Dataset, FrameRow, frame_id

IMAGE_SIZE = 96  # pixels, width and height
CLIP_SPLITS = {
    'indoor_a': 'train',
    'indoor_b': 'val',
    'outdoor_a': 'train',
    'outdoor_b': 'val',
    'outdoor_c': 'hold',
}
FRAME_INDEXES = (0, 10, 20)  # the last one is the frame with no branch
GRAY = 128
BAR_BRIGHTNESS = 230
BAR_LENGTH = 70  # pixels
BAR_WIDTH = 10


def _bar_corners(angle):
    """Return the four corners (pixels, float) of a bar through the image centre at `angle`.

    cv2.boxPoints turns a rotated rectangle (centre, size, angle in degrees) into its corners.
    """
    centre = (IMAGE_SIZE / 2, IMAGE_SIZE / 2)
    return cv2.boxPoints((centre, (BAR_LENGTH, BAR_WIDTH), angle))


def _label_line(corners):
    """Return a YOLO segmentation label line for one polygon given in pixels."""
    normalized = corners / IMAGE_SIZE
    return '0 ' + ' '.join(f'{value:.6f}' for value in normalized.flatten()) + '\n'


def make_tiny_dataset(root):
    """Write the tiny dataset under root and return it as a Dataset.

    Every clip is in clips.csv, manifest.csv and splits.csv, and every frame has an image and a
    label, so the held clip (outdoor_c) is labeled too and must be left out by the code under test.
    """
    dataset = Dataset(root)
    dataset.images_dir.mkdir(parents=True)
    dataset.labels_dir.mkdir(parents=True)

    clips, frames = [], []
    for clip_number, clip in enumerate(CLIP_SPLITS):
        domain = clip.split('_')[0]
        clips.append(ClipRow(clip, domain, 0, '0' * 32))
        for frame_number, index in enumerate(FRAME_INDEXES):
            name = frame_id(clip, index)
            frames.append(FrameRow(name, clip, domain, index, index / 30, 'batch_001'))
            image = np.full((IMAGE_SIZE, IMAGE_SIZE), GRAY, dtype=np.uint8)
            label = ''
            if index != FRAME_INDEXES[-1]:
                # A different angle for every frame of every clip.
                corners = _bar_corners(angle=clip_number * 35 + frame_number * 70)
                cv2.fillPoly(image, [np.round(corners).astype(np.int32)], BAR_BRIGHTNESS)
                label = _label_line(corners)
            cv2.imwrite(str(dataset.image_path(name)), image)
            dataset.label_path(name).write_text(label)

    dataset.write_clips(clips)
    dataset.write_frames(frames)
    dataset.write_splits(CLIP_SPLITS)
    return dataset


def remove_labels(dataset, clips):
    """Delete the label files of every frame of these clips, so they count as not labeled."""
    for row in dataset.read_frames():
        if row.clip in clips:
            dataset.label_path(row.frame_id).unlink()
