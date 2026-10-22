# Training the branch segmentation model

These commands turn recorded `.bag` clips into a trained YOLO26n-seg branch segmentation model.
Run them from `src/perch_perception/`. Nothing outside `training/` imports this folder, so the
training libraries never reach the drone.

Install the extra packages first:

```bash
python -m pip install -r requirements-training.txt
```

## The dataset folder

Everything lives in `~/vespr_dataset/` (`--dataset` picks another folder), outside the repo,
because recorded data never goes into git. `dataset.py` owns its layout:

```
vespr_dataset/
  manifest.csv          one row per extracted frame
  clips.csv             one row per extracted clip: name, domain, size, MD5
  splits.csv            one row per clip: train, val, hold or test
  images/<frame_id>.png left IR, 8-bit, lossless
  labels/<frame_id>.txt YOLO segmentation polygons; present only once labeled
  to_label/<batch>.zip  new, unlabeled frames packed for upload
  runs/<run_name>/      one folder per training run, with its run.json
```

Splits are by clip, never by frame. A new clip goes to `train` or `val` (about 80/20, by a hash
of its name), and each domain's first clip goes to `val`. Edit `splits.csv` by hand to mark a
clip `hold` or `test`. Every command that reads `splits.csv` refuses to run if a clip marked
`hold` or `test` has already been used by a training run.

## What John runs, start to finish

```bash
# Mac
python -m perch_perception.training.extract_frames --domain outdoor ~/vespr_recordings/outdoor
# upload vespr_dataset/to_label/<batch>.zip to the labeling tool, label, export
python -m perch_perception.training.import_labels ~/Downloads/<export>.zip
python -m perch_perception.training.contact_sheet
rsync -a ~/vespr_dataset/ <server>:vespr_dataset/

# server, in tmux
python -m perch_perception.training.train
python -m perch_perception.training.evaluate <run>
python -m perch_perception.training.export_model <run>
```

Each step is described below as it is written.
