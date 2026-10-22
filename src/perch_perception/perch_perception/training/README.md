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

## 1. Extract frames and pack them for labeling

```bash
python -m perch_perception.training.extract_frames --domain outdoor ~/vespr_recordings/outdoor
python -m perch_perception.training.extract_frames --domain indoor FOLDER \
    --hold 2026-10-19_14-03-22
```

Every `.bag` in the folder is replayed, and every 10th frame (`--every`), starting with the
first, is saved as `images/<clip>_<frame_index>.png`: the left IR image, 8-bit, lossless. The
clip's name is the `.bag` file's name. Frames come from the `.bag` only, never the `.mp4`
preview. `--domain` is required and applies to every clip in the folder, so keep indoor and
outdoor clips in separate folders.

Run it again on the same folder whenever new clips arrive. A clip already extracted is skipped
when its MD5 matches; one with the same name but different content is refused, because two
recordings can share a name when the Jetson clock is wrong. Rename the file in that case.

`--hold CLIP` (repeat it for more clips) marks a new clip `hold` in `splits.csv` as it is
extracted. To hold a clip that was extracted earlier, add or edit its row in `splits.csv` by
hand (a clip has no row until its first labels are imported).

If a `.bag` cannot be read to the end (for example an unindexed or truncated file), the command
stops with an error naming the file. Clips before it stay extracted, and the failed clip has no
rows, so the next run tries it again.

At the end, the frames not yet in a batch are zipped into `to_label/batch_<nnn>.zip`, at most 100
per zip (`--batch-size`), with the PNGs at the top of the zip. Upload each zip to the labeling
tool. The batch each frame went into is recorded in `manifest.csv`.
