# Training the branch segmentation model

These commands turn recorded `.bag` clips into a trained YOLO26n-seg branch segmentation model.
Run them from `src/perch_perception/`. Nothing outside `training/` imports this folder, so the
training libraries never reach the drone.

Install the extra packages first (on the 5090 server, install PyTorch before them; see Setting up
the server):

```bash
python -m pip install -r requirements-training.txt
```

## The dataset folder

Everything lives in `~/vespr_dataset/` (`--dataset` picks another folder), outside the repo,
because recorded data never goes into git. `dataset.py` owns its layout:

```
vespr_dataset/
  manifest.csv          one row per extracted frame
  clips.csv             one row per extracted clip: name, domain, size, MD5, branch
  splits.csv            one row per clip: train, val, hold or test
  images/<frame_id>.png left IR, 8-bit, lossless
  labels/<frame_id>.txt YOLO segmentation polygons; present only once labeled
  to_label/<batch>.zip  new, unlabeled frames packed for upload
  runs/<run_name>/      one folder per training run, with its run.json
```

Splits are by clip, never by frame. Clips of the same branch are near-copies too, so every clip of
a branch gets the same split. The branch is the `group` column of `clips.csv`, taken from the site
and branch fields the recorder saved in the clip's `.json` (for example `park/7`). A clip with no
branch is a group of one.

A new clip joins the split its branch already has, whatever it is. A new branch, or a clip with
no branch, goes to `train` or `val` (about 80/20, by a hash of the branch, or of the clip's name
when it has none), and each domain's first clip goes to `val`. Edit `splits.csv` by hand to mark
clips `hold` or `test`, and change every clip of a branch together. Every command that reads
`splits.csv` refuses to run if a clip marked `hold` or `test`, or any clip of its branch, has
already been used by a training run, or if the clips of one branch have different splits.

## What John runs, start to finish

```bash
# Mac
python -m perch_perception.training.extract_frames --domain outdoor ~/vespr_recordings/outdoor
# upload vespr_dataset/to_label/<batch>.zip to the labeling tool, label, export
python -m perch_perception.training.import_labels ~/Downloads/<export>.zip
python -m perch_perception.training.contact_sheet
rsync -au ~/vespr_dataset/ <server>:vespr_dataset/

# server, in tmux
python -m perch_perception.training.train
python -m perch_perception.training.evaluate <run>
python -m perch_perception.training.export_model <run>

# Mac, afterwards: bring the runs back, so the leakage guard knows which clips they used
rsync -au <server>:vespr_dataset/runs/ ~/vespr_dataset/runs/
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

The recorder saves a `.json` of details next to each `.bag`, and extraction reads it:

- A clip marked bad is skipped and listed (`skipped 2 clips marked bad: ...`). It is never
  extracted while it stays marked. A clip marked bad after it was extracted keeps its frames in
  the dataset, with a warning: nothing removes them by itself.
- Its site and branch go into `clips.csv` as the clip's branch: trimmed, spaces squeezed and lower
  case, so `Park ` and `park` are the same site. A branch without a site is the branch alone; a
  site without a branch is no branch. The branch is read once, when the clip is first extracted:
  to change it later, edit `group` in `clips.csv`.
- A clip whose `.json` says the other domain is refused, so an indoor clip in the outdoor folder
  is caught.
- A `.json` that is not valid JSON stops the command with an error naming the file.

Clips recorded before the recorder wrote a `.json` have no branch and are never skipped.

Run it again on the same folder whenever new clips arrive. A clip already extracted is skipped
when its MD5 matches; one with the same name but different content is refused, because two
recordings can share a name if the recorder's counter file was lost and the Jetson clock was wrong.
Rename the file in that case.

`--hold CLIP` (repeat it for more clips) marks a new clip `hold` in `splits.csv` as it is
extracted. Holding one clip of a branch is enough: the branch's other clips join `hold` when their
labels are imported. A clip whose branch already has clips in another split cannot be held, and
neither can a clip marked bad. To hold a clip that was extracted earlier, add or edit its row in
`splits.csv` by hand (a clip has no row until its first labels are imported), with the rows of
every other clip of its branch.

If a `.bag` cannot be read to the end (for example an unindexed or truncated file), the command
stops with an error naming the file. Clips before it stay extracted, and the failed clip has no
rows, so the next run tries it again, and fails on it again. If the clip is beyond saving, set
`"bad": true` in its `.json` (create the file, `{"bad": true}`, for a clip that has none), and
the next run skips it.

At the end, the frames not yet in a batch are zipped into `to_label/batch_<nnn>.zip`, at most 100
per zip (`--batch-size`), with the PNGs at the top of the zip. Upload each zip to the labeling
tool. The batch each frame went into is recorded in `manifest.csv`.

## 2. Label a batch and import it

Upload `to_label/batch_<nnn>.zip` to the labeling tool (CVAT or Roboflow) and draw a polygon
around each branch, class `branch`. **Finish the whole batch before exporting**: open every
frame, and mark the ones with no branch (in Roboflow, "null"), because some tools export an empty
label for a frame nobody looked at, and only a finished batch tells "no branch" apart from "not
labeled yet".

Export the batch:

- **CVAT**: "Ultralytics YOLO Segmentation". CVAT keeps the file names.
- **Roboflow**: "YOLOv8/YOLO11 segmentation". Turn off resize and every other preprocessing and
  augmentation step first, so the polygons match the original frames. Roboflow renames files
  (`<frame>_png.rf.<hash>.txt`); the import undoes that.

Then import the zip, or the folder it unzips to:

```bash
python -m perch_perception.training.import_labels ~/Downloads/<export>.zip
```

Only the `.txt` files inside a `labels` folder are read; `data.yaml`, `train.txt`, images,
`labels.cache` and macOS `__MACOSX` entries are ignored. Any train, valid or test folders the tool
made are ignored too: splits come from `splits.csv` only. Each label is copied to
`labels/<frame_id>.txt`, with Windows line endings, trailing spaces and blank lines cleaned up.

The import is refused, changing nothing, if:

- the export does not cover exactly the frames of one batch (it says how many are missing)
- a label matches no frame in `manifest.csv`, or a frame that was never packed into a batch
- a polygon is malformed: an odd number of coordinates, fewer than 3 points, a value outside 0
  to 1, or a class other than 0
- a clip marked `hold` or `test`, or another clip of its branch, was already used by a training
  run
- the clips of one branch have different splits in `splits.csv`

It prints how many frames came back with no branch (`<empty> of <labels> frames came back with no
branch`): a large number usually means the batch was not finished. Importing a batch again
replaces its labels and says how many. Each clip that now has labels and no row in `splits.csv`
is given a split, as described above: its branch's split, or `train` or `val` for a new branch.
Clips already in `splits.csv`, such as `hold` clips, keep their split. Deleting a clip's row from
`splits.csv` makes it new again, so the next import gives it a split again: to keep a clip held,
change its split, never delete its row.

The import ends with a count of clips per split for each domain, such as `outdoor clips: 9 train,
2 val, 1 hold, 0 test`. It warns when a domain has no clip in `train`: with a single branch, or a
single branch name, every clip of a domain lands in one split and the model learns nothing about
that domain. Give each branch, or each indoor rig setup, its own name when recording.

## 3. Check the labels on a contact sheet

```bash
python -m perch_perception.training.contact_sheet
python -m perch_perception.training.contact_sheet --count 100 --seed 3
```

This draws 50 labeled frames (`--count`), chosen at random, into one grid image,
`contact_sheet.png` in the dataset folder (`--out` writes it somewhere else). Each tile is the
frame with its branch masks filled in green at 40% opacity and outlined, and its frame id
underneath. Open it and look for masks that miss the branch, cover the background or are missing.
Frames labeled as having no branch are sampled too and show with nothing drawn: check that they
really show no branch.

The frames are picked from every labeled frame, whatever its split, `hold` clips included,
because looking at labels is not scoring. The same `--seed` always picks the same frames, so a
sheet can be made again after fixing labels; a new seed shows other frames. A `--count` larger
than the number of labeled frames draws all of them.

## Setting up the server

Training runs on the 5090 server over SSH, inside `tmux`, so a dropped connection does not stop
it. Once, on the server, with Python 3.11:

```bash
git clone <repo url> ~/vespr && cd ~/vespr/src/perch_perception
python3.11 -m venv ~/vespr-venv && source ~/vespr-venv/bin/activate
nvidia-smi                       # "CUDA Version" in the corner must be 13.0 or newer
python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu130
python -m pip install -r requirements-training.txt
python -c "import torch; print(torch.cuda.get_device_name(0))"     # prints the 5090
```

Install PyTorch first, from pytorch.org's own index, so pip does not install a build for another
GPU. The 5090 is a Blackwell GPU, which needs a CUDA 12.8 or newer build. On 2026-10-22,
PyTorch 2.14.1 had CUDA 13.0 (`cu130`) and 13.2 (`cu132`) builds but no 12.8 build (`cu128`
stops at 2.11). If `nvidia-smi` shows a CUDA version below 13.0, update the NVIDIA driver, or
use `cu128`, which installs PyTorch 2.11. This line has not been tried on our server yet.

The first training run downloads the pretrained weights into the current folder
(`yolo26n-seg.pt`, and `weights/yolo26n.pt` on a GPU). `src/perch_perception/.gitignore`
ignores every `.pt` file, so they never go into git and do not mark later runs `dirty`.

Ultralytics sends anonymous usage data by default. To turn that off on a machine, run
`yolo settings sync=False` once.

## Moving the dataset to the server and back

The Mac holds the dataset; the server gets a copy. Before training, send it over with `rsync`,
which copies only what changed:

```bash
rsync -au ~/vespr_dataset/ <server>:vespr_dataset/
```

After training, bring the run folders back:

```bash
rsync -au <server>:vespr_dataset/runs/ ~/vespr_dataset/runs/
```

`-u` skips every file that is newer on the receiving side. Without it, sending the dataset after
pulling a run that was still training would replace that run's final weights on the server with
the Mac's older copy.

Do this every time. The leakage guard reads `runs/*/run.json` to know which clips were used, so a
run left only on the server does not protect its clips on the Mac. Never add `--delete`: it
would remove runs from one side that only the other side has. For the same reason, never delete
a run folder: removing `runs/<run>/`, or its `run.json`, erases the guard's record of which clips
that run used. To free space, delete its weights, but keep at least `run.json`.

## 4. Train

```bash
python -m perch_perception.training.train                      # 320 and 416, on GPU 0
python -m perch_perception.training.train --sizes 416 --device 0
```

Run it on the server inside `tmux` (`tmux new -A -s train`). It trains one run per image size, one
after the other, with the settings in `config/training.yaml` (`--config` picks another file).
`--sizes` replaces the config's `image_sizes`, and `--device` is a GPU index or `cpu`.

Before Ultralytics starts, each run:

- refuses to start if a clip marked `hold` or `test` was used by an earlier run
- stops with an error if the `train` or the `val` clips have no labeled frames yet (`val` frames
  are what Ultralytics scores each epoch on)
- gets a folder `runs/<YYYY-MM-DD>_<size>_<nn>/` in the dataset folder, numbered from `01` per
  day and size, with `train.txt` and `val.txt` (the image paths), `data.yaml` for Ultralytics and
  `run.json`

`run.json` is written before training, so a run that crashes still counts as having used its
clips. It records the run's name and image size, when it was made, the git commit and whether
the working tree had uncommitted changes (`dirty`), the whole config, and the train and val clips
with their labeled frame counts. When training finishes, it also records the Ultralytics and
PyTorch versions and the GPU.

Every run starts from the COCO-pretrained `yolo26n-seg.pt`, never from an earlier run. Ultralytics
downloads it into the current folder on the first run, and on a GPU it also downloads
`weights/yolo26n.pt` for its mixed-precision check (both git-ignored, see Setting up the
server). Ultralytics stops early when its validation fitness (mostly mask mAP) has not
improved for `patience` epochs, and keeps the best epoch as `weights/best.pt`. It saves its
curves, `results.csv` and sample images in the same run folder. It also writes `labels.cache`
in the dataset folder, which it rebuilds when the labels change.

Augmentation: hue and saturation are off, because they do nothing on grayscale IR, and brightness
(`hsv_v`) is 0.6, stronger than Ultralytics' 0.4, for lighting. Contrast is not augmented.
Ultralytics 8.4.184 has no contrast setting of its own; contrast would take an Albumentations
transform passed as `augmentations=`, and installing `albumentations` also installs
`opencv-python-headless` next to our `opencv-python`: two packages that both provide `cv2` and
overwrite each other's files.

The images are 8-bit grayscale PNGs, and Ultralytics loads them with OpenCV as three identical
channels, which is what the pretrained model expects. **Inference on the drone must feed frames
the same way**: the left IR image copied into three identical channels, never a single channel
and never a colour image. Each frame must also be letterboxed to the run's square image size, as
the ONNX file expects: scaled to fit the square, keeping its shape, with the rest padded in gray
(114, as Ultralytics pads it).

To check the code path without a GPU, run `python -m pytest test/test_train.py` on the Mac. It
trains for one epoch on the CPU, on a tiny generated dataset.

## Training on Colab instead

If the server is unavailable, the same command runs on Colab Pro with a GPU runtime. Keep a copy
of `vespr_dataset` in Google Drive, then in a notebook cell:

```python
from google.colab import drive; drive.mount('/content/drive')
!git clone <repo url> /content/vespr
!cp -r "/content/drive/MyDrive/vespr_dataset" /content/vespr_dataset
%cd /content/vespr/src/perch_perception
!pip install -r requirements-training.txt
!python -m perch_perception.training.train --dataset /content/vespr_dataset
!cp -r /content/vespr_dataset/runs/. "/content/drive/MyDrive/vespr_dataset/runs/"
```

Colab already has a CUDA build of PyTorch, so do not install another one. Copy the runs back to
Drive before the runtime ends (the last line), and from there into `~/vespr_dataset/runs/` on the
Mac, for the same reason as `rsync` above.

## 5. Evaluate and compare runs

```bash
python -m perch_perception.training.evaluate 2026-10-22_320_01
python -m perch_perception.training.evaluate 2026-10-22_320_01 2026-10-22_416_01 --overlays 0
```

Run it on the server, after training. Each run named (a folder in `runs/`) is scored with its
`weights/best.pt` on the labeled frames of today's `val` clips: box and mask mAP50 and mAP50-95,
for all frames (`all`), then `indoor` and `outdoor` separately. Each frame is letterboxed to a
square of the run's image size, the only input the exported engine takes. A domain with no
labeled val frames is left out. mAP50 counts a predicted branch as found when its overlap with
the label (intersection over union) is at least 0.5; mAP50-95 averages that over thresholds 0.5
to 0.95.

It prints one table, a line per run and domain. The val split grows as clips are labeled, so
scores saved on different days are not comparable: to compare runs, name them all in one
command, and each is scored again on the same frames. Each run's scores are also saved as
`runs/<run>/scores_<YYYY-MM-DD>.json`, and the split lists and Ultralytics' output as
`runs/<run>/eval_<YYYY-MM-DD>/<all|indoor|outdoor>/`.

It also draws 20 val frames per run (`--overlays`, `0` for none), chosen at random with a fixed
seed, into `runs/<run>/overlays/<frame_id>.png`: the run's predicted masks in red over the labels
in green. `--device` is a GPU index or `cpu`; by default Ultralytics takes the first GPU, or the
CPU if there is none.

Only `val` is scored. Clips marked `hold` or `test` are never scored or drawn, and the command
refuses to run if one of them was used by a training run. Scoring the test split is a separate,
deliberate step at the end of the project.

## 6. Export to ONNX and check it

```bash
python -m perch_perception.training.export_model 2026-10-22_320_01
```

Run it on the server, after choosing a run. It exports the run's `weights/best.pt` to
`weights/best.onnx` in the same run folder, at the run's image size, batch 1, with the ONNX opset
`onnx_opset` from `config/training.yaml` (`--config` picks another file).

Then it checks the file. Up to 10 labeled `val` frames, spread across the `val` clips (never `hold`
or `test`) go through both the ONNX file, with `onnxruntime`, and the original weights. Both get
the same input the engine will get: the frame scaled to fit a square of the run's image size,
the rest padded (letterboxing). For each frame, every predicted branch is merged into one mask
the frame's size, and the two masks are
compared by intersection over union (IoU: the pixels in both masks over the pixels in either;
two empty masks count as 1.0). The check passes when every frame's IoU is at least
`onnx_min_mask_iou` (0.98). The result goes in `runs/<run>/export.json`: the ONNX path, the
opset, each frame's IoU, the lowest, the tolerance and whether it passed. The command exits with
status 1 when the check fails, and refuses to run, like the others, if a clip marked `hold` or
`test` was used by a training run. The frames are taken one `val` clip at a time in turn, frames
that show a branch first, so even a few frames cover every clip and compare real masks.

Both the export and the check run on the CPU, and the ONNX graph is not simplified
(`simplify=False`). Either way round, Ultralytics would `pip install` a missing package without
asking: `onnxslim` to simplify, and `onnxruntime-gpu` to predict with the ONNX file on a GPU.
TensorRT simplifies the graph itself when it builds the engine.

The TensorRT engine is not built here. That is plan task T7.1, done on the Jetson from
`best.onnx`, because an engine only runs on the TensorRT version and GPU it was built for. The
opset must be one the Jetson's TensorRT accepts: TensorRT 10.x accepts opset 17. The Jetson's
TensorRT version has not been checked yet (on 2026-10-22 the Jetson did not answer over SSH), so
check it before T7.1 with `dpkg -l | grep -i tensorrt` on the Jetson, and lower `onnx_opset` if
it is older than 10.
