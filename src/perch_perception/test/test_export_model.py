"""Tests for training/export_model.py, which exports a run to ONNX and checks it.

The export tests use the session's one-epoch CPU run on the tiny dataset (tiny_run in
conftest.py). They add best.onnx and export.json to its run folder and change nothing else.
That model is barely trained, so they check the plumbing, not accuracy.
"""

import json

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from tiny_dataset import make_tiny_dataset  # noqa: E402

from perch_perception.training.dataset import (  # noqa: E402
    DatasetError, LeakageError, labeled_frames)
from perch_perception.training.export_model import (  # noqa: E402
    check_export, check_frames, compare_exports, export_run, main, mask_iou, predicted_mask)
from perch_perception.training.train import load_config  # noqa: E402

VAL_FRAMES = 6  # three labeled frames in each val clip: indoor_b and outdoor_b

# PyTorch warns while Ultralytics exports: its older ONNX exporter is deprecated, and tracing
# (recording the model's operations on one example input) cannot follow Python if-statements.
# Both are expected for a fixed-size, batch 1 export, so they are hidden here.
pytestmark = [
    pytest.mark.filterwarnings('ignore:You are using the legacy TorchScript-based ONNX export'),
    pytest.mark.filterwarnings('ignore:The feature will be removed'),
    pytest.mark.filterwarnings('ignore:Converting a tensor to a Python boolean'),
]


def test_mask_iou_of_identical_masks_is_one():
    mask = np.zeros((4, 4), dtype=bool)
    mask[1:3, 1:3] = True

    assert mask_iou(mask, mask.copy()) == 1.0


def test_mask_iou_of_two_empty_masks_is_one():
    empty = np.zeros((4, 4), dtype=bool)

    assert mask_iou(empty, empty.copy()) == 1.0


def test_mask_iou_of_disjoint_masks_is_zero():
    left = np.zeros((4, 4), dtype=bool)
    left[:, :2] = True

    assert mask_iou(left, ~left) == 0.0


def test_mask_iou_is_the_overlap_over_the_union():
    a = np.zeros((1, 4), dtype=bool)
    a[0, :2] = True  # pixels 0 and 1
    b = np.zeros((1, 4), dtype=bool)
    b[0, 1:] = True  # pixels 1, 2 and 3

    assert mask_iou(a, b) == pytest.approx(1 / 4)


def test_an_exported_run_matches_its_weights(tiny_run):
    pytest.importorskip('onnxruntime')
    dataset, run_dir = tiny_run

    result = check_export(dataset, run_dir.name, load_config())

    saved = json.loads((run_dir / 'export.json').read_text())
    assert saved == result
    assert saved['onnx'] == str(run_dir / 'weights' / 'best.onnx')
    assert (run_dir / 'weights' / 'best.onnx').exists()
    assert saved['opset'] == 17
    assert saved['tolerance'] == 0.98
    assert len(saved['ious']) == VAL_FRAMES
    assert saved['min_iou'] == min(saved['ious'])
    assert saved['passed'] is True


def test_an_export_below_tolerance_fails(tiny_run):
    pytest.importorskip('onnxruntime')
    dataset, run_dir = tiny_run
    config = {**load_config(), 'onnx_min_mask_iou': 1.01}  # no IoU can reach it

    result = check_export(dataset, run_dir.name, config, count=2)

    assert len(result['ious']) == 2
    assert result['passed'] is False


def test_the_command_exits_with_status_one_when_the_check_fails(tiny_run, tmp_path):
    pytest.importorskip('onnxruntime')
    dataset, run_dir = tiny_run
    config_path = tmp_path / 'training.yaml'
    config_path.write_text('onnx_opset: 17\nonnx_min_mask_iou: 1.01\n')

    status = main([run_dir.name, '--dataset', str(dataset.root), '--config', str(config_path)])

    assert status == 1


def test_masks_are_compared_when_the_models_find_something(tiny_run):
    # At Ultralytics' default confidence of 0.25 the barely trained model finds nothing, so every
    # IoU is 1.0 from two empty masks. A very low confidence makes both models predict masks.
    # (At 0.001 every pixel is predicted; at 0.005 the masks cover part of the frame.)
    pytest.importorskip('onnxruntime')
    from ultralytics import YOLO

    dataset, run_dir = tiny_run
    onnx_path = export_run(dataset, run_dir.name, load_config())
    pt_path = run_dir / 'weights' / 'best.pt'
    images = [dataset.image_path(row.frame_id) for row in labeled_frames(dataset, 'val')]

    ious = compare_exports(pt_path, onnx_path, images, image_size=64, conf=0.005)

    assert len(ious) == len(images)
    assert all(0.0 <= iou <= 1.0 for iou in ious)
    pt_model = YOLO(str(pt_path))
    onnx_model = YOLO(str(onnx_path), task='segment')
    found_by_both = [
        predicted_mask(pt_model, image, 64, conf=0.005).any()
        and predicted_mask(onnx_model, image, 64, conf=0.005).any()
        for image in images
    ]
    assert any(found_by_both)


def test_a_predicted_mask_is_the_size_of_the_frame(tiny_run):
    from ultralytics import YOLO

    dataset, run_dir = tiny_run
    image = dataset.image_path(labeled_frames(dataset, 'val')[0].frame_id)

    mask = predicted_mask(YOLO(str(run_dir / 'weights' / 'best.pt')), image, 64, conf=0.005)

    assert mask.dtype == bool
    assert mask.shape == (96, 96)  # the frame's own size, not the 64 pixel model input


def test_the_weights_see_the_same_square_input_as_the_onnx_file(tiny_run, tmp_path):
    # The ONNX file has a fixed square input. By default Ultralytics feeds a wide frame to the
    # .pt weights as a smaller rectangle instead, so the two would not be compared on the same
    # input. 40x160 is wide enough at image size 64 to show it: the rectangle would be 32x64.
    onnx = pytest.importorskip('onnx')
    from ultralytics import YOLO

    dataset, run_dir = tiny_run
    onnx_path = export_run(dataset, run_dir.name, load_config())
    onnx_input = [dim.dim_value
                  for dim in onnx.load(onnx_path).graph.input[0].type.tensor_type.shape.dim]
    image = tmp_path / 'wide.png'
    cv2.imwrite(str(image), np.full((40, 160), 128, dtype=np.uint8))
    pt_model = YOLO(str(run_dir / 'weights' / 'best.pt'))
    pt_inputs = []
    # A forward pre-hook is called with the tensor each time the network runs.
    pt_model.model.register_forward_pre_hook(
        lambda module, args: pt_inputs.append(list(args[0].shape)))

    pt_mask = predicted_mask(pt_model, image, 64, conf=0.005)
    onnx_mask = predicted_mask(YOLO(str(onnx_path), task='segment'), image, 64, conf=0.005)

    assert onnx_input == [1, 3, 64, 64]
    assert pt_inputs == [onnx_input]
    assert pt_mask.shape == onnx_mask.shape == (40, 160)  # the frame's own size


def test_exporting_a_run_that_does_not_exist_is_an_error(tmp_path, capsys):
    dataset = make_tiny_dataset(tmp_path / 'dataset')

    with pytest.raises(DatasetError, match='2026-10-22_320_09'):
        check_export(dataset, '2026-10-22_320_09', load_config())

    status = main(['2026-10-22_320_09', '--dataset', str(dataset.root)])
    assert status == 1
    assert capsys.readouterr().err.startswith('error: ')


def test_checking_with_no_labeled_val_frames_is_an_error(tmp_path):
    dataset = make_tiny_dataset(tmp_path / 'dataset')
    for row in dataset.read_frames():
        if row.clip in ('indoor_b', 'outdoor_b'):
            dataset.label_path(row.frame_id).unlink()

    with pytest.raises(DatasetError, match='no labeled frames in val clips'):
        check_export(dataset, '2026-10-22_320_01', load_config())


def test_exporting_refuses_when_a_held_clip_was_used(tmp_path, capsys):
    dataset = make_tiny_dataset(tmp_path / 'dataset')
    (dataset.runs_dir / 'r1').mkdir(parents=True)
    (dataset.runs_dir / 'r1' / 'run.json').write_text(
        json.dumps({'clips': {'train': ['outdoor_c'], 'val': []}}))  # outdoor_c is held

    with pytest.raises(LeakageError, match='outdoor_c'):
        check_export(dataset, 'r1', load_config())

    status = main(['r1', '--dataset', str(dataset.root)])
    assert status == 1
    assert capsys.readouterr().err.startswith('error: ')


def test_the_export_command_prints_an_error_for_a_config_it_cannot_read(tmp_path, capsys):
    dataset = make_tiny_dataset(tmp_path / 'dataset')

    status = main(['2026-10-22_320_01', '--dataset', str(dataset.root),
                   '--config', str(tmp_path / 'missing.yaml')])

    assert status == 1
    error = capsys.readouterr().err
    assert error.startswith('error: ') and 'missing.yaml' in error


def test_the_check_frames_are_spread_across_the_val_clips_branches_first(tmp_path):
    dataset = make_tiny_dataset(tmp_path / 'dataset')
    val = labeled_frames(dataset, 'val')  # indoor_b's three frames, then outdoor_b's

    assert {row.clip for row in check_frames(dataset, val, count=2)} == {'indoor_b', 'outdoor_b'}
    # Frame 20 of each clip is labeled as showing no branch, so those two come last.
    assert [row.frame_id for row in check_frames(dataset, val, count=10)] == [
        'indoor_b_000000', 'outdoor_b_000000', 'indoor_b_000010', 'outdoor_b_000010',
        'indoor_b_000020', 'outdoor_b_000020']
