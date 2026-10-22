"""Tests for training/train.py, which trains YOLO26n-seg on the labeled frames.

Most tests only prepare a run on the tiny dataset (tiny_dataset.py), which needs no Ultralytics.
The last one uses the session's one-epoch CPU run (tiny_run in conftest.py).
"""

import json
import shutil
import subprocess
from datetime import date

import pytest

pytest.importorskip('cv2')

from tiny_dataset import make_tiny_dataset  # noqa: E402

from perch_perception.training.dataset import (  # noqa: E402
    DatasetError, LeakageError, check_splits)
from perch_perception.training.train import (  # noqa: E402
    PACKAGE_DIR, git_state, load_config, main, prepare_run, run_name)

DAY = date(2026, 10, 22)


def remove_labels(dataset, clips):
    """Delete the label files of every frame of these clips, so they count as not labeled."""
    for row in dataset.read_frames():
        if row.clip in clips:
            dataset.label_path(row.frame_id).unlink()


def test_run_names_count_up_per_day_and_size(tmp_path):
    (tmp_path / '2026-10-22_320_01').mkdir()

    assert run_name(tmp_path, DAY, 320) == '2026-10-22_320_02'
    assert run_name(tmp_path, DAY, 416) == '2026-10-22_416_01'
    assert run_name(tmp_path, date(2026, 10, 23), 320) == '2026-10-23_320_01'


def test_git_state_outside_a_repository_is_unknown_and_dirty(tmp_path):
    assert git_state(tmp_path) == {'commit': 'unknown', 'dirty': True}


def test_downloaded_weights_do_not_make_a_run_dirty(tmp_path):
    # A fresh repository with the package's .gitignore, committed, then the files Ultralytics
    # downloads into the working folder.
    shutil.copy(PACKAGE_DIR / '.gitignore', tmp_path)
    git = ['git', '-C', str(tmp_path), '-c', 'user.name=test', '-c', 'user.email=test@example.com',
           '-c', 'commit.gpgsign=false']
    for command in (['init', '-q'], ['add', '.gitignore'], ['commit', '-q', '-m', 'start']):
        subprocess.run(git + command, check=True)
    (tmp_path / 'yolo26n-seg.pt').write_bytes(b'weights')
    (tmp_path / 'weights').mkdir()
    (tmp_path / 'weights' / 'yolo26n.pt').write_bytes(b'weights')

    assert git_state(tmp_path)['dirty'] is False


def test_a_run_records_its_clips_and_settings_before_training(tmp_path):
    dataset = make_tiny_dataset(tmp_path / 'dataset')
    config = load_config()

    run_dir = prepare_run(dataset, config, image_size=320, day=DAY)

    assert run_dir == dataset.runs_dir / '2026-10-22_320_01'
    record = json.loads((run_dir / 'run.json').read_text())
    assert record['name'] == '2026-10-22_320_01'
    assert record['image_size'] == 320
    assert record['clips'] == {'train': ['indoor_a', 'outdoor_a'],
                               'val': ['indoor_b', 'outdoor_b']}
    assert 'outdoor_c' not in (run_dir / 'run.json').read_text()
    assert record['frames'] == {'train': 6, 'val': 6}  # three labeled frames per clip
    assert record['config'] == config
    assert set(record['git']) == {'commit', 'dirty'}
    assert record['created'].startswith('20')
    assert (run_dir / 'data.yaml').exists()
    assert 'outdoor_c' not in (run_dir / 'train.txt').read_text()
    assert 'outdoor_c' not in (run_dir / 'val.txt').read_text()


def test_a_prepared_run_marks_its_clips_as_used(tmp_path):
    dataset = make_tiny_dataset(tmp_path / 'dataset')
    prepare_run(dataset, load_config(), image_size=320, day=DAY)

    splits = dataset.read_splits()
    dataset.write_splits({**splits, 'indoor_a': 'hold'})

    with pytest.raises(LeakageError, match='indoor_a'):
        check_splits(dataset)


def test_training_with_nothing_labeled_is_an_error(tmp_path):
    dataset = make_tiny_dataset(tmp_path / 'dataset')
    shutil.rmtree(dataset.labels_dir)

    with pytest.raises(DatasetError, match='no labeled frames in train clips'):
        prepare_run(dataset, load_config(), image_size=320, day=DAY)

    assert not dataset.runs_dir.exists()


def test_training_with_no_labeled_val_frames_is_an_error(tmp_path):
    dataset = make_tiny_dataset(tmp_path / 'dataset')
    remove_labels(dataset, ['indoor_b', 'outdoor_b'])

    with pytest.raises(DatasetError, match='no labeled frames in val clips'):
        prepare_run(dataset, load_config(), image_size=320, day=DAY)

    assert not dataset.runs_dir.exists()


def test_training_refuses_to_start_when_a_held_clip_was_used(tmp_path):
    dataset = make_tiny_dataset(tmp_path / 'dataset')
    earlier = dataset.runs_dir / '2026-10-21_320_01'
    earlier.mkdir(parents=True)
    (earlier / 'run.json').write_text(json.dumps({'clips': {'train': ['outdoor_c'], 'val': []}}))

    with pytest.raises(LeakageError, match='outdoor_c'):
        prepare_run(dataset, load_config(), image_size=320, day=DAY)

    assert [path.name for path in dataset.runs_dir.iterdir()] == ['2026-10-21_320_01']


def test_the_train_command_prints_an_error_when_nothing_is_labeled(tmp_path, capsys):
    dataset = make_tiny_dataset(tmp_path / 'dataset')
    shutil.rmtree(dataset.labels_dir)

    status = main(['--dataset', str(dataset.root), '--device', 'cpu'])

    assert status == 1
    assert capsys.readouterr().err.startswith('error: ')


@pytest.mark.parametrize('text', [None, 'epochs: [1, 2\n'])  # no file, then a YAML syntax error
def test_the_train_command_prints_an_error_for_a_config_it_cannot_read(tmp_path, capsys, text):
    config_path = tmp_path / 'training.yaml'
    if text is not None:
        config_path.write_text(text)
    dataset = make_tiny_dataset(tmp_path / 'dataset')

    with pytest.raises(DatasetError, match='training.yaml'):
        load_config(config_path)
    status = main(['--dataset', str(dataset.root), '--config', str(config_path)])

    assert status == 1
    error = capsys.readouterr().err
    assert error.startswith('error: ') and 'training.yaml' in error
    assert not dataset.runs_dir.exists()  # stopped before any run was prepared


def test_a_one_epoch_run_saves_weights_and_versions(tiny_run):
    dataset, run_dir = tiny_run

    assert (run_dir / 'weights' / 'best.pt').exists()
    record = json.loads((run_dir / 'run.json').read_text())
    assert record['versions']['ultralytics'] == '8.4.184'
    assert record['versions']['torch']
    assert record['versions']['gpu'] == 'cpu'
    assert record['image_size'] == 64
