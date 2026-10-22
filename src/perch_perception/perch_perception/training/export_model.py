"""Export a run's best weights to ONNX and check that the ONNX file predicts the same masks.

    python -m perch_perception.training.export_model 2026-10-22_320_01

The ONNX file is what the Jetson builds its TensorRT engine from (plan task T7.1, done on the
Jetson). It is exported at the run's image size, batch 1, with the opset in
config/training.yaml, to runs/<run>/weights/best.onnx. Then up to 10 val frames, spread across
the val clips, go through both the ONNX file (with onnxruntime) and the original weights, and
each frame's two masks are compared. The result goes in runs/<run>/export.json, and the command
exits with status 1 when a frame's masks overlap less than onnx_min_mask_iou from the config.

Ultralytics does the export and the predictions; ultralytics is imported inside the functions
that use them, so the rest of this module loads without it.
"""

import argparse
import json
import sys
from itertools import zip_longest
from pathlib import Path

import numpy as np

from perch_perception.training.dataset import (
    DEFAULT_DATASET, Dataset, DatasetError, check_splits, labeled_frames, parse_label, run_files)
from perch_perception.training.train import load_config


def mask_iou(a, b):
    """Return the intersection over union of two boolean masks of the same shape.

    Intersection over union is the number of pixels in both masks divided by the number in
    either. Two empty masks agree perfectly, so they give 1.0.
    """
    union = np.logical_or(a, b).sum()
    if union == 0:
        return 1.0
    return float(np.logical_and(a, b).sum() / union)


def export_run(dataset, run, config):
    """Export a run's best weights to ONNX at its image size, batch 1. Return the ONNX path.

    Ultralytics writes the file next to the weights: runs/<run>/weights/best.onnx.
    """
    _, weights, image_size = run_files(dataset, run)

    # Imported here, not at the top, so the drone never needs this heavy library.
    from ultralytics import YOLO

    path = YOLO(str(weights)).export(
        format='onnx',
        imgsz=image_size,
        batch=1,
        opset=config['onnx_opset'],
        # Simplifying the graph needs onnxslim, which is not installed, and Ultralytics would
        # pip-install it without asking. TensorRT simplifies the graph itself when it builds.
        simplify=False,
        device='cpu',
    )
    return Path(path)


def predicted_mask(model, image_path, image_size, conf):
    """Return one model's prediction for a frame as a single boolean mask the frame's size.

    Every predicted branch with confidence at least conf is OR-ed into the one mask, so the two
    models are compared on which pixels are branch, not on how they split them into branches.
    """
    # Given the path, Ultralytics loads the frame as training did: three identical channels.
    # retina_masks makes Ultralytics return masks at the frame's own size, not the model's.
    # rect=False is letterboxing to the export's fixed square input: the frame is scaled to fit
    # an image_size square and the rest padded. The ONNX file, and the TensorRT engine built from
    # it, only take that square. By default Ultralytics gives .pt weights a smaller rectangle
    # instead, and the comparison would then mix that difference in with the export's.
    result = model.predict(str(image_path), imgsz=image_size, conf=conf, device='cpu',
                           retina_masks=True, rect=False, verbose=False)[0]
    if result.masks is None:  # nothing found
        return np.zeros(result.orig_shape, dtype=bool)
    return result.masks.data.cpu().numpy().any(axis=0)


def compare_exports(pt_path, onnx_path, images, image_size, conf=0.25):
    """Predict each image with the original weights and with the ONNX file. Return the IoUs.

    One mask_iou per image, in order. conf is the confidence a prediction needs to count; 0.25
    is Ultralytics' own default. Both run on the CPU: on a GPU, Ultralytics would pip-install
    onnxruntime-gpu without asking.
    """
    # Imported here, not at the top, so the drone never needs this heavy library.
    from ultralytics import YOLO

    pt_model = YOLO(str(pt_path))
    # The task is given, so Ultralytics does not have to guess it from the ONNX file.
    onnx_model = YOLO(str(onnx_path), task='segment')
    return [
        mask_iou(predicted_mask(pt_model, image, image_size, conf),
                 predicted_mask(onnx_model, image, image_size, conf))
        for image in images
    ]


def _round_robin(frames):
    """Return frames reordered one clip at a time: each clip's first frame, then each second.

    Clips are taken in the order they first appear, and each clip's frames keep their order.
    """
    by_clip = {}  # clip: its frames; a dict keeps the order clips were added
    for row in frames:
        by_clip.setdefault(row.clip, []).append(row)
    # zip_longest takes one frame from every clip per step, padding a clip that has run out
    # with None, which is skipped.
    return [row for step in zip_longest(*by_clip.values()) for row in step if row is not None]


def check_frames(dataset, frames, count):
    """Return up to count of these labeled frames to check an export on, spread across clips.

    Frames whose label shows a branch come first, so the masks are compared, not only empty
    frames. They are taken round-robin by clip, in manifest order, so even a few frames cover
    every clip. Frames labeled as showing no branch fill what is left, in the same way.
    """
    with_branch = []
    without_branch = []
    for row in frames:
        if parse_label(dataset.label_path(row.frame_id).read_text()):
            with_branch.append(row)
        else:
            without_branch.append(row)
    return (_round_robin(with_branch) + _round_robin(without_branch))[:count]


def check_export(dataset, run, config, count=10):
    """Export a run, compare it with its weights on up to count val frames and save the result.

    Returns {'onnx', 'opset', 'ious', 'min_iou', 'tolerance', 'passed'}, also written to
    runs/<run>/export.json. It passes when every frame's IoU is at least onnx_min_mask_iou.
    The frames are today's labeled val frames, never hold or test, chosen by check_frames. Raises
    LeakageError if a held-out clip was used by a run, and DatasetError if val has no labeled
    frames or the run is missing.
    """
    check_splits(dataset)
    val = labeled_frames(dataset, 'val')
    if not val:
        raise DatasetError(f'{dataset.root} has no labeled frames in val clips yet, so there is '
                           'nothing to compare on. Label a batch from a val clip '
                           '(see splits.csv).')
    run_dir, weights, image_size = run_files(dataset, run)

    onnx_path = export_run(dataset, run, config)
    images = [dataset.image_path(row.frame_id) for row in check_frames(dataset, val, count)]
    ious = compare_exports(weights, onnx_path, images, image_size)
    tolerance = config['onnx_min_mask_iou']
    result = {
        'onnx': str(onnx_path),
        'opset': config['onnx_opset'],
        'ious': ious,
        'min_iou': min(ious),
        'tolerance': tolerance,
        'passed': min(ious) >= tolerance,
    }
    (run_dir / 'export.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run', metavar='RUN', help='run name, a folder in <dataset>/runs')
    parser.add_argument('--dataset', type=Path, default=DEFAULT_DATASET,
                        help=f'dataset folder (default {DEFAULT_DATASET})')
    parser.add_argument('--config', type=Path, default=None,
                        help='training settings (default config/training.yaml)')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    dataset = Dataset(args.dataset)
    try:
        config = load_config(args.config)
        result = check_export(dataset, args.run, config)
    except DatasetError as error:
        print(f'error: {error}', file=sys.stderr)
        return 1

    verdict = 'passed' if result['passed'] else 'FAILED'
    print(f'wrote {result["onnx"]} (opset {result["opset"]})')
    print(f'{verdict}: lowest mask IoU {result["min_iou"]:.3f} over {len(result["ious"])} val '
          f'frames, tolerance {result["tolerance"]}')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
