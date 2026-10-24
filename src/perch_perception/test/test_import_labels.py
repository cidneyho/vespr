"""Tests for training/import_labels.py, which brings a labeled batch back into the dataset.

Every test builds a small dataset folder under tmp_path: two indoor clips with two frames each,
all four packed in batch_001. The rows are written directly, with no images and no extraction.
Exports are written as zips (or a folder) laid out the way the labeling tools write them.
"""

import json
import zipfile

import pytest

from perch_perception.training.dataset import (
    ClipRow, Dataset, DatasetError, FrameRow, LeakageError, frame_id)
from perch_perception.training.import_labels import (
    import_batch, main, original_stem, read_export)

CLIP = '2026-10-19_14-03-22'
OTHER_CLIP = '2026-10-19_14-05-10'
FRAMES = [frame_id(CLIP, 0), frame_id(CLIP, 10), frame_id(OTHER_CLIP, 0),
          frame_id(OTHER_CLIP, 10)]
POLYGON = '0 0.1 0.2 0.3 0.2 0.3 0.4\n'


def make_dataset(root, group='', other_group=''):
    """A dataset with two indoor clips, both in clips.csv, and their four frames in batch_001.

    group and other_group are the branches of CLIP and OTHER_CLIP; '' is none.
    """
    dataset = Dataset(root)
    dataset.write_clips([ClipRow(CLIP, 'indoor', 100, 'f' * 32, group),
                         ClipRow(OTHER_CLIP, 'indoor', 100, 'e' * 32, other_group)])
    dataset.write_frames([
        FrameRow(frame_id(clip, index), clip, 'indoor', index, index / 30, 'batch_001')
        for clip in (CLIP, OTHER_CLIP) for index in (0, 10)])
    return dataset


def write_export(path, files):
    """Write an export zip holding {name inside the zip: text}."""
    with zipfile.ZipFile(path, 'w') as archive:
        for name, text in files.items():
            archive.writestr(name, text)
    return path


def full_export(path, text=POLYGON):
    """An export zip with one label per frame of batch_001, laid out the way CVAT writes it."""
    return write_export(path, {f'labels/train/{name}.txt': text for name in FRAMES})


def snapshot(dataset):
    """Return every file in the dataset folder as {relative path: bytes}, to show nothing moved."""
    return {str(path.relative_to(dataset.root)): path.read_bytes()
            for path in sorted(dataset.root.rglob('*')) if path.is_file()}


def test_a_finished_batch_is_imported_into_labels(tmp_path):
    dataset = make_dataset(tmp_path / 'dataset')

    summary = import_batch(dataset, full_export(tmp_path / 'export.zip'))

    assert summary.batch == 'batch_001'
    assert summary.labels == 4
    assert summary.empty == 0
    assert summary.replaced == 0
    for name in FRAMES:
        assert dataset.label_path(name).read_text() == POLYGON
    # The first clip of the domain goes to val; the other's name hashes to train.
    assert summary.new_splits == {CLIP: 'val', OTHER_CLIP: 'train'}
    assert dataset.read_splits() == {CLIP: 'val', OTHER_CLIP: 'train'}


def test_roboflow_names_are_matched_to_the_original_frame(tmp_path):
    assert original_stem('clip_000010_png.rf.3f2a9c.txt'.removesuffix('.txt')) == 'clip_000010'
    assert original_stem('clip_000010') == 'clip_000010'  # CVAT keeps the name

    # Roboflow exports a folder per split of its own, each with a labels folder inside.
    export = tmp_path / 'roboflow'
    for number, name in enumerate(FRAMES):
        folder = export / ('train' if number % 2 else 'valid') / 'labels'
        folder.mkdir(parents=True, exist_ok=True)
        (folder / f'{name}_png.rf.{number:032x}.txt').write_text(POLYGON)
    dataset = make_dataset(tmp_path / 'dataset')

    summary = import_batch(dataset, export)

    assert summary.labels == 4
    assert all(dataset.is_labeled(name) for name in FRAMES)


def test_files_outside_a_labels_folder_and_macos_metadata_are_ignored(tmp_path):
    files = {f'labels/train/{name}.txt': POLYGON for name in FRAMES}
    files.update({
        'data.yaml': 'names:\n  0: branch\n',
        'train.txt': 'images/train/x.png\n',
        f'images/train/{FRAMES[0]}.png': 'not really a png',
        'labels/train/labels.cache': 'binary cache',
        f'__MACOSX/labels/train/._{FRAMES[0]}.txt': '\x00\x05\x16\x07 resource fork',
    })
    export = write_export(tmp_path / 'export.zip', files)

    assert sorted(read_export(export)) == sorted(FRAMES)
    summary = import_batch(make_dataset(tmp_path / 'dataset'), export)
    assert summary.labels == 4


def test_windows_line_endings_and_blank_lines_are_accepted(tmp_path):
    text = '0 0.1 0.2 0.3 0.2 0.3 0.4  \r\n\r\n0 0.5 0.5 0.6 0.5 0.6 0.6\r\n'
    dataset = make_dataset(tmp_path / 'dataset')

    import_batch(dataset, full_export(tmp_path / 'export.zip', text))

    # Stored with one polygon per line and plain newlines, so every tool reads it the same way.
    assert dataset.label_path(FRAMES[0]).read_text() == (
        '0 0.1 0.2 0.3 0.2 0.3 0.4\n0 0.5 0.5 0.6 0.5 0.6 0.6\n')


@pytest.mark.parametrize('as_folder', [False, True])
def test_a_label_saved_with_a_byte_order_mark_is_read(tmp_path, as_folder):
    # Some Windows editors start a UTF-8 file with a byte order mark (BOM), U+FEFF.
    text = '\ufeff' + POLYGON
    if as_folder:
        export = tmp_path / 'export'
        (export / 'labels').mkdir(parents=True)
        for name in FRAMES:
            (export / 'labels' / f'{name}.txt').write_text(text, encoding='utf-8')
    else:
        export = full_export(tmp_path / 'export.zip', text)
    dataset = make_dataset(tmp_path / 'dataset')

    import_batch(dataset, export)

    assert dataset.label_path(FRAMES[0]).read_text() == POLYGON  # stored without the BOM


def test_an_export_missing_frames_of_its_batch_is_refused(tmp_path):
    dataset = make_dataset(tmp_path / 'dataset')
    export = write_export(tmp_path / 'export.zip',
                          {f'labels/train/{name}.txt': POLYGON for name in FRAMES[:2]})
    before = snapshot(dataset)

    with pytest.raises(DatasetError, match='missing 2 of 4 frames of batch_001'):
        import_batch(dataset, export)

    assert snapshot(dataset) == before


def test_an_export_spanning_two_batches_is_refused(tmp_path):
    dataset = make_dataset(tmp_path / 'dataset')
    later = FrameRow(frame_id(OTHER_CLIP, 20), OTHER_CLIP, 'indoor', 20, 20 / 30, 'batch_002')
    dataset.write_frames(dataset.read_frames() + [later])
    files = {f'labels/train/{name}.txt': POLYGON for name in FRAMES + [later.frame_id]}
    export = write_export(tmp_path / 'export.zip', files)
    before = snapshot(dataset)

    with pytest.raises(DatasetError, match='batch_001, batch_002'):
        import_batch(dataset, export)

    assert snapshot(dataset) == before


def test_a_label_for_an_unknown_frame_is_refused(tmp_path):
    dataset = make_dataset(tmp_path / 'dataset')
    files = {f'labels/train/{name}.txt': POLYGON for name in FRAMES}
    files['labels/train/2026-01-01_00-00-00_000000.txt'] = POLYGON
    export = write_export(tmp_path / 'export.zip', files)
    before = snapshot(dataset)

    with pytest.raises(DatasetError, match='2026-01-01_00-00-00_000000'):
        import_batch(dataset, export)

    assert snapshot(dataset) == before


def test_a_label_for_a_frame_that_was_never_packed_is_refused(tmp_path):
    dataset = make_dataset(tmp_path / 'dataset')
    unpacked = FrameRow(frame_id(OTHER_CLIP, 20), OTHER_CLIP, 'indoor', 20, 20 / 30, '')
    dataset.write_frames(dataset.read_frames() + [unpacked])
    files = {f'labels/train/{name}.txt': POLYGON for name in FRAMES + [unpacked.frame_id]}
    export = write_export(tmp_path / 'export.zip', files)
    before = snapshot(dataset)

    with pytest.raises(DatasetError, match='never packed'):
        import_batch(dataset, export)

    assert snapshot(dataset) == before


def test_a_clip_missing_from_clips_csv_is_refused(tmp_path):
    # Extraction writes a clip's frames before its clips.csv row, so a crash between the two
    # leaves frames whose clip is not in clips.csv.
    dataset = make_dataset(tmp_path / 'dataset')
    dataset.write_clips([row for row in dataset.read_clips() if row.clip != OTHER_CLIP])
    export = full_export(tmp_path / 'export.zip')
    before = snapshot(dataset)

    with pytest.raises(DatasetError, match=f'{OTHER_CLIP} is not in clips.csv'):
        import_batch(dataset, export)

    assert snapshot(dataset) == before


def test_one_bad_polygon_refuses_the_whole_import(tmp_path):
    dataset = make_dataset(tmp_path / 'dataset')
    files = {f'labels/train/{name}.txt': POLYGON for name in FRAMES}
    files[f'labels/train/{FRAMES[3]}.txt'] = '0 0.1 0.2 0.3 0.2\n'  # only 2 points
    export = write_export(tmp_path / 'export.zip', files)
    before = snapshot(dataset)

    with pytest.raises(DatasetError, match=f'{FRAMES[3]}.*fewer than 3 points'):
        import_batch(dataset, export)

    assert snapshot(dataset) == before


def test_an_import_is_refused_when_a_held_clip_was_already_used_by_a_run(tmp_path):
    dataset = make_dataset(tmp_path / 'dataset')
    dataset.write_splits({CLIP: 'test'})
    run_dir = dataset.runs_dir / 'run_1'
    run_dir.mkdir(parents=True)
    (run_dir / 'run.json').write_text(json.dumps({'clips': {'train': [CLIP], 'val': []}}))
    export = full_export(tmp_path / 'export.zip')
    before = snapshot(dataset)

    with pytest.raises(LeakageError):
        import_batch(dataset, export)

    assert snapshot(dataset) == before


def test_new_clips_get_train_or_val_and_held_clips_stay_held(tmp_path):
    dataset = make_dataset(tmp_path / 'dataset')
    dataset.write_splits({OTHER_CLIP: 'hold'})

    summary = import_batch(dataset, full_export(tmp_path / 'export.zip'))

    assert summary.new_splits == {CLIP: 'val'}
    assert dataset.read_splits() == {CLIP: 'val', OTHER_CLIP: 'hold'}


def test_a_new_clip_joins_the_split_of_its_branch_even_when_held(tmp_path):
    dataset = make_dataset(tmp_path / 'dataset', group='lab/1', other_group='lab/1')
    dataset.write_splits({OTHER_CLIP: 'hold'})

    summary = import_batch(dataset, full_export(tmp_path / 'export.zip'))

    assert summary.new_splits == {CLIP: 'hold'}


def test_new_clips_of_one_branch_in_one_import_get_the_same_split(tmp_path):
    dataset = make_dataset(tmp_path / 'dataset', group='lab/1', other_group='lab/1')

    summary = import_batch(dataset, full_export(tmp_path / 'export.zip'))

    # The first indoor clip goes to val, and its branch goes with it.
    assert summary.new_splits == {CLIP: 'val', OTHER_CLIP: 'val'}


def test_the_clips_of_each_split_are_counted_per_domain(tmp_path, capsys):
    dataset = make_dataset(tmp_path / 'dataset')
    dataset.write_splits({OTHER_CLIP: 'hold'})

    main([str(full_export(tmp_path / 'export.zip')), '--dataset', str(dataset.root)])

    assert 'indoor clips: 0 train, 1 val, 1 hold, 0 test' in capsys.readouterr().out


def test_a_domain_with_no_train_clip_is_warned_about(tmp_path, capsys):
    # One indoor branch means one indoor split: all val, so the model learns nothing indoors.
    dataset = make_dataset(tmp_path / 'dataset', group='lab/1', other_group='lab/1')

    main([str(full_export(tmp_path / 'export.zip')), '--dataset', str(dataset.root)])

    assert 'warning: no indoor clip is in train' in capsys.readouterr().out


def test_reimporting_a_batch_replaces_its_labels_and_says_so(tmp_path):
    dataset = make_dataset(tmp_path / 'dataset')
    import_batch(dataset, full_export(tmp_path / 'first.zip'))
    corrected = '0 0.5 0.5 0.6 0.5 0.6 0.6\n'

    summary = import_batch(dataset, full_export(tmp_path / 'second.zip', corrected))

    assert summary.replaced == 4
    assert summary.new_splits == {}  # the clips keep the splits they were given the first time
    assert dataset.label_path(FRAMES[0]).read_text() == corrected


def test_empty_labels_are_counted(tmp_path, capsys):
    dataset = make_dataset(tmp_path / 'dataset')
    files = {f'labels/train/{name}.txt': POLYGON for name in FRAMES}
    files[f'labels/train/{FRAMES[1]}.txt'] = ''  # labeled, and shows no branch
    export = write_export(tmp_path / 'export.zip', files)

    status = main([str(export), '--dataset', str(dataset.root)])

    assert status == 0
    assert '1 of 4 frames came back with no branch' in capsys.readouterr().out
    assert dataset.is_labeled(FRAMES[1])
    assert dataset.label_path(FRAMES[1]).read_text() == ''
