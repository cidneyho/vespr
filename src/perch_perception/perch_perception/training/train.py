"""Train YOLO26n-seg on the labeled frames, one run per image size.

    python -m perch_perception.training.train
    python -m perch_perception.training.train --sizes 320 --device cpu

Runs on the 5090 server. Each run gets a folder runs/<YYYY-MM-DD>_<size>_<nn>/ in the dataset
folder, holding the split lists, the Ultralytics data YAML and run.json: the clips and settings
it used and the code's git commit. run.json is written before training starts, so even a run
that crashes counts as having used its clips (see check_splits). Ultralytics then saves its
weights, curves and results.csv into the same folder.

Settings come from config/training.yaml. Ultralytics does the training; this module only feeds
it the run's files and settings. torch and ultralytics are imported inside train_run only, so
the rest of this module loads without them.
"""

import argparse
import json
import subprocess
import sys
from datetime import date, datetime
from pathlib import Path

import yaml

from perch_perception.training.dataset import (
    DEFAULT_DATASET, Dataset, DatasetError, check_splits, labeled_frames, write_data_yaml)

PACKAGE_DIR = Path(__file__).resolve().parents[2]  # src/perch_perception
DEFAULT_CONFIG = PACKAGE_DIR / 'config' / 'training.yaml'


def load_config(path=None):
    """Return the training settings from a YAML file, config/training.yaml by default.

    Raises DatasetError naming the file if it cannot be read or is not valid YAML.
    """
    path = DEFAULT_CONFIG if path is None else Path(path).expanduser()
    try:
        return yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as error:
        raise DatasetError(f'{path} cannot be read as training settings: {error}') from None


def run_name(runs_dir, day, image_size):
    """Return the next free run name for this day and image size: <YYYY-MM-DD>_<size>_<nn>."""
    number = 1
    while (Path(runs_dir) / f'{day.isoformat()}_{image_size}_{number:02d}').exists():
        number += 1
    return f'{day.isoformat()}_{image_size}_{number:02d}'


def git_state(repo_dir):
    """Return the code's git commit and whether the working tree has uncommitted changes.

    Outside a git repository (or without git) the commit is 'unknown' and the tree counts as
    dirty, because nothing proves the code matches a commit.
    """
    try:
        commit = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=repo_dir,
                                capture_output=True, text=True)
        status = subprocess.run(['git', 'status', '--porcelain'], cwd=repo_dir,
                                capture_output=True, text=True)
    except FileNotFoundError:  # git is not installed
        return {'commit': 'unknown', 'dirty': True}
    if commit.returncode != 0 or status.returncode != 0:
        return {'commit': 'unknown', 'dirty': True}
    return {'commit': commit.stdout.strip(), 'dirty': status.stdout.strip() != ''}


def prepare_run(dataset, config, image_size, day):
    """Create a run folder with its split lists, data YAML and run.json. Return its path.

    Raises LeakageError if a held-out clip was used before, and DatasetError if the train or
    val split has no labeled frames. Nothing is written when either is raised.
    """
    check_splits(dataset)
    train = labeled_frames(dataset, 'train')
    val = labeled_frames(dataset, 'val')
    if not train:
        raise DatasetError(f'{dataset.root} has no labeled frames in train clips yet. '
                           'Import a labeled batch first.')
    if not val:
        raise DatasetError(f'{dataset.root} has no labeled frames in val clips yet, so a run '
                           'cannot be scored. Label a batch from a val clip (see splits.csv).')

    name = run_name(dataset.runs_dir, day, image_size)
    run_dir = dataset.runs_dir / name
    write_data_yaml(dataset, run_dir, train, val)
    record = {
        'name': name,
        'image_size': image_size,
        'created': datetime.now().isoformat(timespec='seconds'),
        'git': git_state(PACKAGE_DIR),
        'config': config,
        # Sorted lists, because used_clips reads them back from every run's run.json.
        'clips': {'train': sorted({row.clip for row in train}),
                  'val': sorted({row.clip for row in val})},
        'frames': {'train': len(train), 'val': len(val)},
    }
    (run_dir / 'run.json').write_text(json.dumps(record, indent=2) + '\n')
    return run_dir


def train_run(run_dir, config, device):
    """Train a prepared run with Ultralytics, then add the library versions to its run.json.

    device is what Ultralytics takes: a GPU index such as '0', or 'cpu'.
    """
    # Imported here, not at the top, so the drone never needs these heavy libraries.
    import torch
    import ultralytics
    from ultralytics import YOLO

    run_json = run_dir / 'run.json'
    record = json.loads(run_json.read_text())
    model = YOLO(config['model'])
    model.train(
        data=str(run_dir / 'data.yaml'),
        imgsz=record['image_size'],
        # Ultralytics saves into project/name. The project must be absolute, or Ultralytics
        # puts it under its own runs folder instead.
        project=str(run_dir.parent.absolute()),
        name=run_dir.name,
        exist_ok=True,  # the folder exists already: prepare_run made it
        device=device,
        deterministic=True,
        epochs=config['epochs'],
        patience=config['patience'],
        batch=config['batch'],
        seed=config['seed'],
        workers=config['workers'],
        hsv_h=config['hsv_h'],
        hsv_s=config['hsv_s'],
        hsv_v=config['hsv_v'],
    )

    on_gpu = device != 'cpu' and torch.cuda.is_available()
    record['versions'] = {
        'ultralytics': ultralytics.__version__,
        'torch': torch.__version__,
        'gpu': torch.cuda.get_device_name(0) if on_gpu else 'cpu',
    }
    run_json.write_text(json.dumps(record, indent=2) + '\n')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--dataset', type=Path, default=DEFAULT_DATASET,
                        help=f'dataset folder (default {DEFAULT_DATASET})')
    parser.add_argument('--config', type=Path, default=None,
                        help='training settings (default config/training.yaml)')
    parser.add_argument('--sizes', type=int, nargs='+', default=None,
                        help='image sizes, one run each (default image_sizes in the config)')
    parser.add_argument('--device', default='0',
                        help="GPU index, or 'cpu' (default 0)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    dataset = Dataset(args.dataset)
    try:
        config = load_config(args.config)
        for image_size in args.sizes or config['image_sizes']:
            run_dir = prepare_run(dataset, config, image_size, date.today())
            print(f'training {run_dir}')
            train_run(run_dir, config, args.device)
            print(f'finished {run_dir}')
    except DatasetError as error:
        print(f'error: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
