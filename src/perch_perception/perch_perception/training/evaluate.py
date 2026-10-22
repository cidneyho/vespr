"""Score trained runs on today's val split, overall and per domain, and compare them in one table.

    python -m perch_perception.training.evaluate 2026-10-22_320_01
    python -m perch_perception.training.evaluate 2026-10-22_320_01 2026-10-22_416_01

Each run's best weights (runs/<run>/weights/best.pt) are scored on the labeled frames of today's
val clips: box and mask mAP50 and mAP50-95, for all frames, then indoor and outdoor separately.
Scores saved on different days are not comparable, because the val split grows, so every run
named is scored again and the table compares them on the same frames. The scores go in
runs/<run>/scores_<YYYY-MM-DD>.json, and overlays of predictions (red) over labels (green) for a
sample of val frames in runs/<run>/overlays/.

Only val is scored. Clips marked hold or test are never read here; scoring the test split is a
separate, deliberate step at the end of the project. Ultralytics does the scoring; ultralytics
is imported inside the functions that use it, so the rest of this module loads without it.
"""

import argparse
import json
import random
import sys
from datetime import date
from pathlib import Path

import cv2

from perch_perception.training.contact_sheet import draw_polygons
from perch_perception.training.dataset import (
    DEFAULT_DATASET, DOMAINS, Dataset, DatasetError, check_splits, labeled_frames, parse_label,
    run_files, write_data_yaml)

LABEL_COLOR = (0, 255, 0)  # green, in OpenCV's blue, green, red order
PREDICTION_COLOR = (0, 0, 255)  # red
SCORE_KEYS = ('box_map50', 'box_map', 'mask_map50', 'mask_map')
TABLE_HEADER = ('run', 'domain', 'frames', 'box mAP50', 'box mAP50-95', 'mask mAP50',
                'mask mAP50-95')


def score_run(dataset, run, domain, day, device=None):
    """Score a run's best weights on today's labeled val frames of one domain (None: all).

    Returns {'run', 'domain', 'frames', 'box_map50', 'box_map', 'mask_map50', 'mask_map'}, with
    domain 'all' for None, or None when the domain has no labeled val frames. The split lists
    and Ultralytics' output go in runs/<run>/eval_<YYYY-MM-DD>/<all|indoor|outdoor>/. device is
    what Ultralytics takes, a GPU index such as '0' or 'cpu'; None lets it pick.
    """
    run_dir, weights, image_size = run_files(dataset, run)
    val = labeled_frames(dataset, 'val', domain)
    if not val:
        return None
    # The data YAML needs a train list too, so it gets today's train frames, but only the val
    # list is scored. Hold and test clips are in neither list.
    train = labeled_frames(dataset, 'train', domain)
    domain_name = domain or 'all'
    eval_dir = run_dir / f'eval_{day.isoformat()}' / domain_name
    write_data_yaml(dataset, eval_dir, train, val)

    # Imported here, not at the top, so the drone never needs this heavy library.
    from ultralytics import YOLO

    # rect=False is letterboxing to the export's fixed square input: each frame is scaled to fit
    # an image_size square and the rest padded. The ONNX file, and the TensorRT engine built from
    # it, only take that square. By default Ultralytics scores .pt weights on rectangles shaped
    # to each batch of frames instead, an input the drone never gives the model.
    metrics = YOLO(str(weights)).val(
        data=str(eval_dir / 'data.yaml'),
        imgsz=image_size,
        rect=False,
        split='val',
        plots=False,
        # Ultralytics saves into project/name. The project must be absolute, or Ultralytics
        # puts it under its own runs folder instead.
        project=str(eval_dir.parent.absolute()),
        name=eval_dir.name,
        exist_ok=True,  # the folder exists already: write_data_yaml made it
        device=device,
    )
    # Mean average precision: mAP50 counts a prediction as right when its intersection over
    # union with a label is at least 0.5. mAP (mAP50-95) averages that over 0.5 to 0.95.
    return {
        'run': run,
        'domain': domain_name,
        'frames': len(val),
        'box_map50': float(metrics.box.map50),
        'box_map': float(metrics.box.map),
        'mask_map50': float(metrics.seg.map50),
        'mask_map': float(metrics.seg.map),
    }


def evaluate_runs(dataset, runs, day, device=None):
    """Score every run for all frames, indoor and outdoor, and save each run's scores.

    Returns one row (see score_run) per run and domain, in the order given; a domain with no
    labeled val frames has no row. Each run's rows are also written to
    runs/<run>/scores_<YYYY-MM-DD>.json. Raises LeakageError if a held-out clip was used by a
    run, and DatasetError if val has no labeled frames or a run is missing.
    """
    check_splits(dataset)
    if not labeled_frames(dataset, 'val'):
        raise DatasetError(f'{dataset.root} has no labeled frames in val clips yet, so there is '
                           'nothing to score. Label a batch from a val clip (see splits.csv).')
    # Every run is checked before the first is scored, so a mistyped name fails at once.
    for run in runs:
        run_files(dataset, run)

    rows = []
    for run in runs:
        run_rows = []
        for domain in (None, *DOMAINS):
            row = score_run(dataset, run, domain, day, device)
            if row is not None:
                run_rows.append(row)
        scores_json = dataset.runs_dir / run / f'scores_{day.isoformat()}.json'
        scores_json.write_text(json.dumps(run_rows, indent=2) + '\n')
        rows.extend(run_rows)
    return rows


def format_table(rows):
    """Return rows as a fixed-width table: a header, then one line per run and domain.

    Scores have 3 decimals. Text columns are left-aligned and number columns right-aligned.
    """
    lines = [TABLE_HEADER] + [
        (row['run'], row['domain'], str(row['frames']))
        + tuple(f'{row[key]:.3f}' for key in SCORE_KEYS)
        for row in rows
    ]
    # Each column is as wide as its widest cell. zip(*lines) turns the rows into columns.
    widths = [max(len(cell) for cell in column) for column in zip(*lines)]
    text = []
    for cells in lines:
        padded = [cell.ljust(width) if column < 2 else cell.rjust(width)
                  for column, (cell, width) in enumerate(zip(cells, widths))]
        text.append('  '.join(padded))
    return '\n'.join(text)


def save_overlays(dataset, run, count=20, seed=0, device=None):
    """Draw a run's predictions (red) over the labels (green) for a sample of val frames.

    Frames are drawn at random from today's labeled val frames, never hold or test; the same
    seed picks the same frames. Each is written to runs/<run>/overlays/<frame_id>.png at its
    own size. Returns the paths written, in manifest order.
    """
    run_dir, weights, image_size = run_files(dataset, run)
    val = labeled_frames(dataset, 'val')
    # A random sample without repeats, from a generator seeded with `seed`, as on the contact
    # sheet. Sorting the picked positions keeps the frames in manifest order.
    picked = random.Random(seed).sample(range(len(val)), min(count, len(val)))
    frames = [val[position] for position in sorted(picked)]

    # Imported here, not at the top, so the drone never needs this heavy library.
    from ultralytics import YOLO

    model = YOLO(str(weights))
    out_dir = run_dir / 'overlays'
    out_dir.mkdir(exist_ok=True)
    paths = []
    for row in frames:
        image_path = dataset.image_path(row.frame_id)
        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise DatasetError(f'{image_path} cannot be read as an image')
        labels = parse_label(dataset.label_path(row.frame_id).read_text())
        # Given the path, Ultralytics loads the frame as training did: three identical channels.
        # rect=False letterboxes it to the export's fixed square input, as in score_run.
        result = model.predict(str(image_path), imgsz=image_size, rect=False, device=device,
                               verbose=False)[0]
        # masks.xyn is each predicted mask's outline, normalized to 0 to 1 like a label, so
        # draw_polygons draws both the same way. A mask too small for an outline comes back
        # with no points, and is left out.
        predictions = [] if result.masks is None else [
            polygon for polygon in result.masks.xyn if len(polygon) > 0]
        drawn = draw_polygons(image, labels, LABEL_COLOR)
        drawn = draw_polygons(drawn, predictions, PREDICTION_COLOR)
        path = out_dir / f'{row.frame_id}.png'
        if not cv2.imwrite(str(path), drawn):
            raise DatasetError(f'{path} cannot be written')
        paths.append(path)
    return paths


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+', metavar='RUN',
                        help='run names, the folders in <dataset>/runs')
    parser.add_argument('--dataset', type=Path, default=DEFAULT_DATASET,
                        help=f'dataset folder (default {DEFAULT_DATASET})')
    parser.add_argument('--overlays', type=int, default=20,
                        help='val frames to draw per run, 0 for none (default 20)')
    parser.add_argument('--device', default=None,
                        help="GPU index, or 'cpu' (default: the first GPU, or the CPU)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    dataset = Dataset(args.dataset)
    try:
        rows = evaluate_runs(dataset, args.runs, date.today(), args.device)
        # The table is printed before the overlays are drawn, so an overlay error cannot hide it.
        print(format_table(rows))
        if args.overlays > 0:
            for run in args.runs:
                paths = save_overlays(dataset, run, count=args.overlays, device=args.device)
                print(f'wrote {len(paths)} overlays to {dataset.runs_dir / run / "overlays"}')
    except DatasetError as error:
        print(f'error: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
