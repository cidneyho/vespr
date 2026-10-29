"""Tests for training/evaluate.py, which scores trained runs on today's val split.

Most tests use the session's one-epoch CPU run on the tiny dataset (tiny_run in conftest.py).
That model is barely trained, so these tests check which frames are scored and the shape and
range of the results, not accuracy. tiny_run is shared, so a test that changes the dataset
copies it first.
"""

import json
import shutil
from contextlib import contextmanager
from datetime import date

import pytest

cv2 = pytest.importorskip('cv2')

from tiny_dataset import make_tiny_dataset, remove_labels  # noqa: E402

from perch_perception.training.dataset import (  # noqa: E402
    Dataset, DatasetError, LeakageError, labeled_frames)
from perch_perception.training.evaluate import (  # noqa: E402
    evaluate_runs, format_table, main, save_overlays, score_run)

DAY = date(2026, 10, 22)
SCORES = ('box_map50', 'box_map', 'mask_map50', 'mask_map')
HELD_CLIP = 'outdoor_c'  # the tiny dataset's hold clip


def widen_val_images(dataset):
    """Stretch every val frame to 160x40, wider than the run's 64 pixel square input.

    The labels are normalized to the image size, so they still fit the stretched frames.
    """
    for row in labeled_frames(dataset, 'val'):
        path = dataset.image_path(row.frame_id)
        cv2.imwrite(str(path), cv2.resize(cv2.imread(str(path), cv2.IMREAD_UNCHANGED), (160, 40)))


@contextmanager
def network_input_sizes():
    """Record the height and width of every image batch Ultralytics predicts on, in the block.

    A global forward pre-hook is called with the input of every PyTorch module that runs, so it
    sees the input even when Ultralytics builds the model out of reach of the test. Only
    AutoBackend is recorded: Ultralytics runs every prediction through that wrapper, but not the
    test inputs it gives a copy of the network to count its operations.
    """
    from torch.nn.modules.module import register_module_forward_pre_hook

    sizes = []

    def record(module, args):
        if type(module).__name__ == 'AutoBackend':
            sizes.append(list(args[0].shape[2:]))

    handle = register_module_forward_pre_hook(record)
    try:
        yield sizes
    finally:
        handle.remove()


def test_a_run_is_scored_overall_and_per_domain(tiny_run):
    dataset, run_dir = tiny_run

    rows = evaluate_runs(dataset, [run_dir.name], DAY, device='cpu')

    assert [row['domain'] for row in rows] == ['all', 'indoor', 'outdoor']
    # Three labeled frames in each val clip: indoor_b and outdoor_b.
    assert [row['frames'] for row in rows] == [6, 3, 3]
    for row in rows:
        assert row['run'] == run_dir.name
        for score in SCORES:
            assert 0 <= row[score] <= 1


def test_a_domain_with_no_val_frames_is_skipped(tiny_run, tmp_path):
    original, run_dir = tiny_run
    dataset = Dataset(shutil.copytree(original.root, tmp_path / 'dataset'))
    remove_labels(dataset, ['indoor_b'])

    assert score_run(dataset, run_dir.name, 'indoor', DAY, device='cpu') is None
    rows = evaluate_runs(dataset, [run_dir.name], DAY, device='cpu')
    assert [row['domain'] for row in rows] == ['all', 'outdoor']


def test_held_clips_are_never_scored(tiny_run):
    dataset, run_dir = tiny_run

    evaluate_runs(dataset, [run_dir.name], DAY, device='cpu')

    lists = sorted((run_dir / f'eval_{DAY.isoformat()}').glob('*/*.txt'))
    assert [path.relative_to(run_dir).as_posix() for path in lists] == [
        'eval_2026-10-22/all/train.txt', 'eval_2026-10-22/all/val.txt',
        'eval_2026-10-22/indoor/train.txt', 'eval_2026-10-22/indoor/val.txt',
        'eval_2026-10-22/outdoor/train.txt', 'eval_2026-10-22/outdoor/val.txt']
    for path in lists:
        assert HELD_CLIP not in path.read_text()


def test_scores_are_saved_with_the_date(tiny_run):
    dataset, run_dir = tiny_run

    rows = evaluate_runs(dataset, [run_dir.name], DAY, device='cpu')

    saved = json.loads((run_dir / 'scores_2026-10-22.json').read_text())
    assert saved == rows


def test_the_table_has_one_line_per_run_and_domain():
    rows = [
        {'run': '2026-10-22_320_01', 'domain': domain, 'frames': 6, 'box_map50': 0.5,
         'box_map': 0.25, 'mask_map50': 0.123456, 'mask_map': 1.0}
        for domain in ('all', 'indoor', 'outdoor')
    ] + [
        {'run': '2026-10-22_416_01', 'domain': 'all', 'frames': 6, 'box_map50': 0.0,
         'box_map': 0.0, 'mask_map50': 0.0, 'mask_map': 0.0}
    ]

    lines = format_table(rows).splitlines()

    assert len(lines) == 1 + len(rows)  # a header, then one line per row
    assert len({len(line) for line in lines}) == 1  # fixed width: every line as long
    assert lines[1].split() == ['2026-10-22_320_01', 'all', '6', '0.500', '0.250', '0.123',
                                '1.000']
    assert lines[4].split()[:2] == ['2026-10-22_416_01', 'all']


def test_overlays_are_written_for_val_frames(tiny_run):
    dataset, run_dir = tiny_run

    paths = save_overlays(dataset, run_dir.name, count=4, seed=0, device='cpu')

    assert len(paths) == 4
    val_frames = {row.frame_id for row in dataset.read_frames()
                  if row.clip in ('indoor_b', 'outdoor_b')}
    for path in paths:
        assert path.parent == run_dir / 'overlays'
        assert path.stem in val_frames
        image = cv2.imread(str(path))
        assert image.shape == (96, 96, 3)  # the frame's own size, in colour


def test_scoring_letterboxes_frames_to_the_exports_square_input(tiny_run, tmp_path):
    # By default Ultralytics scores .pt weights on smaller rectangles, but the deployed engine
    # only takes the run's square, so the scores must come from that square.
    original, run_dir = tiny_run
    dataset = Dataset(shutil.copytree(original.root, tmp_path / 'dataset'))
    widen_val_images(dataset)

    with network_input_sizes() as sizes:
        score_run(dataset, run_dir.name, None, DAY, device='cpu')

    assert sizes and all(size == [64, 64] for size in sizes)


def test_overlays_letterbox_frames_to_the_exports_square_input(tiny_run, tmp_path):
    original, run_dir = tiny_run
    dataset = Dataset(shutil.copytree(original.root, tmp_path / 'dataset'))
    widen_val_images(dataset)

    with network_input_sizes() as sizes:
        paths = save_overlays(dataset, run_dir.name, count=2, device='cpu')

    assert sizes == [[64, 64], [64, 64]]
    assert cv2.imread(str(paths[0])).shape == (40, 160, 3)  # drawn at the frame's own size


def test_the_scores_table_is_printed_even_when_an_overlay_cannot_be_written(
        tiny_run, tmp_path, capsys):
    original, run_dir = tiny_run
    dataset = Dataset(shutil.copytree(original.root, tmp_path / 'dataset'))
    # A folder where an overlay image should go, so writing that overlay fails. Overlays that
    # other tests drew in tiny_run's folder were copied too, so they are removed first.
    overlays = dataset.runs_dir / run_dir.name / 'overlays'
    shutil.rmtree(overlays, ignore_errors=True)
    first_val_frame = labeled_frames(dataset, 'val')[0].frame_id
    (overlays / f'{first_val_frame}.png').mkdir(parents=True)

    status = main([run_dir.name, '--dataset', str(dataset.root), '--device', 'cpu',
                   '--overlays', '6'])

    assert status == 1
    output = capsys.readouterr()
    assert 'box mAP50' in output.out  # the table's header
    assert output.err.startswith('error: ')


def test_evaluating_refuses_when_a_held_clip_was_used(tmp_path, capsys):
    dataset = make_tiny_dataset(tmp_path / 'dataset')
    (dataset.runs_dir / 'r1').mkdir(parents=True)
    (dataset.runs_dir / 'r1' / 'run.json').write_text(
        json.dumps({'clips': {'train': [HELD_CLIP], 'val': []}}))

    with pytest.raises(LeakageError, match=HELD_CLIP):
        evaluate_runs(dataset, ['r1'], DAY)

    status = main(['r1', '--dataset', str(dataset.root)])
    assert status == 1
    assert capsys.readouterr().err.startswith('error: ')


def test_evaluating_a_run_that_does_not_exist_is_an_error(tmp_path, capsys):
    dataset = make_tiny_dataset(tmp_path / 'dataset')

    with pytest.raises(DatasetError, match='2026-10-22_320_09'):
        score_run(dataset, '2026-10-22_320_09', None, DAY)

    status = main(['2026-10-22_320_09', '--dataset', str(dataset.root)])
    assert status == 1
    assert capsys.readouterr().err.startswith('error: ')


def test_evaluating_a_run_without_best_weights_is_an_error(tmp_path):
    dataset = make_tiny_dataset(tmp_path / 'dataset')
    (dataset.runs_dir / '2026-10-22_320_01').mkdir(parents=True)

    with pytest.raises(DatasetError, match='best.pt'):
        score_run(dataset, '2026-10-22_320_01', None, DAY)


def test_evaluating_with_no_labeled_val_frames_is_an_error(tmp_path):
    dataset = make_tiny_dataset(tmp_path / 'dataset')
    remove_labels(dataset, ['indoor_b', 'outdoor_b'])

    with pytest.raises(DatasetError, match='no labeled frames in val clips'):
        evaluate_runs(dataset, ['2026-10-22_320_01'], DAY)
