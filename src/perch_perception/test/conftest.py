"""Fixtures shared by several test files.

tiny_run trains a model once per test session, because even one epoch on the CPU takes a while.
Tests that use it must not change its files; copy them first.
"""

from datetime import date

import pytest


@pytest.fixture(scope='session')
def tiny_run(tmp_path_factory):
    """Return (dataset, run_dir) for a one-epoch CPU run on the tiny dataset, at image size 64.

    The model is built from Ultralytics' yolo26n-seg.yaml, untrained, so nothing is downloaded.
    Skipped when ultralytics is not installed.
    """
    pytest.importorskip('ultralytics')
    from perch_perception.training.train import load_config, prepare_run, train_run
    from tiny_dataset import make_tiny_dataset

    dataset = make_tiny_dataset(tmp_path_factory.mktemp('tiny') / 'dataset')
    config = {**load_config(), 'model': 'yolo26n-seg.yaml', 'epochs': 1, 'batch': 4,
              'workers': 0}
    run_dir = prepare_run(dataset, config, image_size=64, day=date(2026, 10, 22))
    train_run(run_dir, config, device='cpu')
    return dataset, run_dir
