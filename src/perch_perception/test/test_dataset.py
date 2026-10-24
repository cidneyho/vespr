"""Tests for training/dataset.py, which owns the dataset folder, its splits and the leakage guard.

Every test builds its own small dataset folder under tmp_path. No camera and no images.
"""

import hashlib
import json
from pathlib import Path

import pytest
import yaml

from perch_perception.training.dataset import (
    ClipRow,
    Dataset,
    DatasetError,
    FrameRow,
    LeakageError,
    assign_split,
    check_splits,
    file_md5,
    frame_id,
    labeled_frames,
    parse_label,
    write_data_yaml,
)

CLIP = '2026-10-19_14-03-22'
OTHER_CLIP = '2026-10-19_15-10-05'


def make_dataset(root, clips, groups=None):
    """Write clips.csv and splits.csv for clips given as {clip: (domain, split)}.

    groups, {clip: group}, gives clips a branch; the others have none.
    """
    groups = groups or {}
    dataset = Dataset(root)
    dataset.write_clips([ClipRow(clip, domain, 100, 'f' * 32, groups.get(clip, ''))
                         for clip, (domain, _) in clips.items()])
    dataset.write_splits({clip: split for clip, (_, split) in clips.items()})
    return dataset


def frame_row(clip, frame_index, domain='indoor'):
    """A manifest row for one frame of a clip."""
    return FrameRow(frame_id(clip, frame_index), clip, domain, frame_index,
                    frame_index / 30, '')


def write_label(dataset, row, text='0 0.1 0.2 0.3 0.2 0.3 0.4\n'):
    """Mark a frame as labeled by writing its label file."""
    dataset.labels_dir.mkdir(parents=True, exist_ok=True)
    dataset.label_path(row.frame_id).write_text(text)


def write_run(dataset, name, train, val):
    """Write the run.json a training run leaves behind, listing the clips it used."""
    run_dir = dataset.runs_dir / name
    run_dir.mkdir(parents=True)
    (run_dir / 'run.json').write_text(json.dumps({'clips': {'train': train, 'val': val}}))


def test_frame_ids_pad_the_index_to_six_digits():
    assert frame_id('2026-10-19_14-03-22', 7) == '2026-10-19_14-03-22_000007'


def test_csv_files_round_trip(tmp_path):
    dataset = Dataset(tmp_path)
    frames = [
        FrameRow(frame_id(CLIP, 0), CLIP, 'indoor', 0, 0.0, 'batch_001'),
        FrameRow(frame_id(CLIP, 10), CLIP, 'indoor', 10, 0.1 + 0.2, ''),
    ]
    clips = [ClipRow(CLIP, 'indoor', 123456789, '0123456789abcdef0123456789abcdef', 'park/7',
                     '0-12.5'),
             ClipRow(OTHER_CLIP, 'indoor', 5, 'f' * 32)]
    splits = {CLIP: 'hold'}

    dataset.write_frames(frames)
    dataset.write_clips(clips)
    dataset.write_splits(splits)

    assert dataset.read_frames() == frames
    assert dataset.read_clips() == clips
    assert dataset.read_splits() == splits


def test_a_clips_csv_from_before_branches_reads_as_clips_without_a_branch(tmp_path):
    (tmp_path / 'clips.csv').write_text(f'clip,domain,size,md5\n{CLIP},indoor,100,{"f" * 32}\n')

    assert Dataset(tmp_path).read_clips() == [ClipRow(CLIP, 'indoor', 100, 'f' * 32, '', '')]


def test_a_missing_csv_reads_as_empty(tmp_path):
    assert Dataset(tmp_path).read_frames() == []
    assert Dataset(tmp_path).read_clips() == []
    assert Dataset(tmp_path).read_splits() == {}


def test_a_splits_csv_saved_with_a_byte_order_mark_is_read(tmp_path):
    # Excel and some editors start a UTF-8 file with a byte order mark (BOM), U+FEFF.
    dataset = make_dataset(tmp_path, {CLIP: ('indoor', 'hold')})
    splits_csv = tmp_path / 'splits.csv'
    splits_csv.write_text('\ufeff' + splits_csv.read_text())

    assert dataset.read_splits() == {CLIP: 'hold'}


def write_all_csvs(dataset):
    """Write one row to each of manifest.csv, clips.csv and splits.csv."""
    dataset.write_frames([frame_row(CLIP, 0)])
    dataset.write_clips([ClipRow(CLIP, 'indoor', 100, 'f' * 32)])
    dataset.write_splits({CLIP: 'train'})


@pytest.mark.parametrize('name, read', [
    ('manifest.csv', Dataset.read_frames),
    ('clips.csv', Dataset.read_clips),
    ('splits.csv', Dataset.read_splits),
])
def test_a_csv_with_its_header_row_deleted_is_an_error_naming_the_file(tmp_path, name, read):
    dataset = Dataset(tmp_path)
    write_all_csvs(dataset)
    path = tmp_path / name
    path.write_text(''.join(path.read_text().splitlines(keepends=True)[1:]))

    with pytest.raises(DatasetError, match=name):
        read(dataset)


def test_a_csv_with_a_value_that_is_not_a_number_is_an_error_naming_the_file(tmp_path):
    dataset = Dataset(tmp_path)
    write_all_csvs(dataset)
    clips_csv = tmp_path / 'clips.csv'
    clips_csv.write_text(clips_csv.read_text().replace(',100,', ',a lot,'))

    with pytest.raises(DatasetError, match='clips.csv'):
        dataset.read_clips()


def test_a_dataset_under_the_home_folder_has_its_tilde_expanded():
    assert '~' not in str(Dataset('~/vespr_dataset').root)


def test_the_md5_of_a_file_larger_than_one_block_is_lowercase_hex(tmp_path):
    data = b'bag data' * 400_000  # about 3 MB, so it is read in several 1 MB blocks
    path = tmp_path / 'clip.bag'
    path.write_bytes(data)

    assert file_md5(path) == hashlib.md5(data).hexdigest()


def test_a_frames_image_and_label_live_in_flat_folders(tmp_path):
    dataset = Dataset(tmp_path)
    name = frame_id(CLIP, 7)

    assert dataset.image_path(name) == tmp_path / 'images' / f'{name}.png'
    assert dataset.label_path(name) == tmp_path / 'labels' / f'{name}.txt'


def test_parse_label_reads_one_polygon_per_line():
    polygons = parse_label('0 0.1 0.2 0.3 0.2 0.3 0.4\n')
    assert len(polygons) == 1 and polygons[0].shape == (3, 2)


def test_parse_label_keeps_the_coordinates_as_x_y_pairs():
    polygons = parse_label('0 0.1 0.2 0.3 0.2 0.3 0.4\n0 0.5 0.5 0.6 0.5 0.6 0.6 0.5 0.6\n')

    assert len(polygons) == 2
    assert polygons[0].tolist() == [[0.1, 0.2], [0.3, 0.2], [0.3, 0.4]]
    assert polygons[1].shape == (4, 2)


def test_an_empty_label_means_no_branch():
    assert parse_label('') == []


@pytest.mark.parametrize('line', [
    '1 0.1 0.2 0.3 0.2 0.3 0.4',      # class other than 0
    '0 0.1 0.2 0.3 0.2 0.3',          # odd number of coordinates
    '0 0.1 0.2 0.3 0.2',              # fewer than 3 points
    '0 0.1 0.2 0.3 1.2 0.3 0.4',      # outside 0 to 1
    '0 0.1 abc 0.3 0.2 0.3 0.4',      # not a number
])
def test_parse_label_rejects_a_bad_line(line):
    with pytest.raises(DatasetError):
        parse_label(line)


def clip_rows(*rows):
    """Return {clip: ClipRow} for rows given as (clip, domain) or (clip, domain, group)."""
    return {row[0]: ClipRow(row[0], row[1], 100, 'f' * 32, *row[2:]) for row in rows}


def test_the_first_clip_of_a_domain_goes_to_val():
    assert assign_split('a', {}, clip_rows(('a', 'indoor'))) == 'val'


def test_later_clips_are_split_by_a_hash_of_the_name():
    names = [f'2026-10-19_14-{i // 60:02d}-{i % 60:02d}' for i in range(200)]
    clips = clip_rows(('v', 'indoor'), *[(name, 'indoor') for name in names])
    splits = {'v': 'val'}

    first = [assign_split(name, splits, clips) for name in names]
    second = [assign_split(name, splits, clips) for name in names]

    assert first == second
    assert set(first) == {'train', 'val'}
    assert 0.1 <= first.count('val') / len(first) <= 0.3


def test_hold_clips_do_not_count_as_a_domains_val_clip():
    clips = clip_rows(('a', 'indoor'), ('b', 'indoor'))

    assert assign_split('b', {'a': 'hold'}, clips) == 'val'


def test_a_val_clip_of_another_domain_does_not_count():
    clips = clip_rows(('a', 'indoor'), ('b', 'outdoor'))

    assert assign_split('b', {'a': 'val'}, clips) == 'val'


@pytest.mark.parametrize('split', ['train', 'val', 'hold', 'test'])
def test_a_clip_joins_the_split_its_branch_already_has(split):
    # Clips of one branch are near-copies, like frames of one clip, so they share a split. Here
    # the domain has no val clip, so without the branch b would go to val: the val case is only
    # there for completeness.
    clips = clip_rows(('a', 'indoor', 'lab/1'), ('b', 'indoor', 'lab/1'))

    assert assign_split('b', {'a': split}, clips) == split


def test_the_same_branch_name_in_the_other_domain_is_another_branch():
    clips = clip_rows(('a', 'outdoor', '1'), ('b', 'indoor', '1'))

    assert assign_split('b', {'a': 'train'}, clips) == 'val'


def test_the_clips_of_a_new_branch_get_the_same_split_from_a_hash_of_the_branch():
    groups = [f'park/{i}' for i in range(100)]
    clips = clip_rows(('v', 'outdoor'),
                      *[(f'{group}-{take}', 'outdoor', group) for group in groups
                        for take in (1, 2)])

    first = [assign_split(f'{group}-1', {'v': 'val'}, clips) for group in groups]
    second = [assign_split(f'{group}-2', {'v': 'val'}, clips) for group in groups]

    assert first == second
    assert set(first) == {'train', 'val'}


@pytest.mark.parametrize('split', ['Val', 'holdout', ''])
def test_an_unknown_split_in_splits_csv_is_an_error(tmp_path, split):
    dataset = make_dataset(tmp_path, {CLIP: ('indoor', split)})

    with pytest.raises(DatasetError, match='splits.csv'):
        dataset.read_splits()


def test_a_split_for_a_clip_not_in_clips_csv_is_an_error(tmp_path):
    dataset = make_dataset(tmp_path, {CLIP: ('indoor', 'train')})
    dataset.write_splits({CLIP: 'train', OTHER_CLIP: 'val'})

    with pytest.raises(DatasetError, match=OTHER_CLIP):
        dataset.read_splits()


def test_a_clip_listed_twice_in_splits_csv_is_an_error(tmp_path):
    dataset = make_dataset(tmp_path, {CLIP: ('indoor', 'train')})
    splits_csv = tmp_path / 'splits.csv'
    splits_csv.write_text(splits_csv.read_text() + f'{CLIP},test\n')

    with pytest.raises(DatasetError, match=CLIP):
        dataset.read_splits()


@pytest.mark.parametrize('split', ['hold', 'test'])
def test_a_held_clip_used_by_a_run_is_reported_as_leakage(tmp_path, split):
    dataset = make_dataset(tmp_path, {CLIP: ('indoor', split), OTHER_CLIP: ('indoor', 'val')})
    write_run(dataset, 'r1', train=[CLIP], val=[OTHER_CLIP])

    with pytest.raises(LeakageError, match=CLIP):
        check_splits(dataset)


def test_every_leaked_clip_is_named(tmp_path):
    dataset = make_dataset(tmp_path, {CLIP: ('indoor', 'hold'), OTHER_CLIP: ('outdoor', 'test')})
    write_run(dataset, 'r1', train=[CLIP], val=[])
    write_run(dataset, 'r2', train=[], val=[OTHER_CLIP])

    with pytest.raises(LeakageError) as error:
        check_splits(dataset)

    assert CLIP in str(error.value) and OTHER_CLIP in str(error.value)


def test_clips_no_run_has_used_can_be_held(tmp_path):
    dataset = make_dataset(tmp_path, {CLIP: ('indoor', 'train'), OTHER_CLIP: ('indoor', 'hold')})
    write_run(dataset, 'r1', train=[CLIP], val=[])

    assert check_splits(dataset) is None


def test_a_held_clip_is_leakage_when_a_run_used_another_clip_of_its_branch(tmp_path):
    # The trained clip has no split row, as after its row was deleted by hand, so the check that
    # a branch's clips share one split cannot see it.
    dataset = make_dataset(tmp_path, {CLIP: ('outdoor', 'hold')},
                           groups={CLIP: 'park/7', OTHER_CLIP: 'park/7'})
    dataset.write_clips(dataset.read_clips() + [ClipRow(OTHER_CLIP, 'outdoor', 100, 'f' * 32,
                                                        'park/7')])
    write_run(dataset, 'r1', train=[OTHER_CLIP], val=[])

    with pytest.raises(LeakageError, match=CLIP):
        check_splits(dataset)


def test_clips_of_one_branch_in_different_splits_are_an_error(tmp_path):
    dataset = make_dataset(tmp_path, {CLIP: ('outdoor', 'train'), OTHER_CLIP: ('outdoor', 'hold')},
                           groups={CLIP: 'park/7', OTHER_CLIP: 'park/7'})

    with pytest.raises(DatasetError) as error:
        check_splits(dataset)

    message = str(error.value)
    assert 'park/7' in message and CLIP in message and OTHER_CLIP in message


def test_clips_of_different_branches_or_no_branch_may_be_in_different_splits(tmp_path):
    clips = {'a': ('outdoor', 'train'), 'b': ('outdoor', 'hold'), 'c': ('outdoor', 'val'),
             'd': ('outdoor', 'train'), 'e': ('indoor', 'val')}
    dataset = make_dataset(tmp_path, clips, groups={'a': 'park/7', 'b': 'park/8', 'e': 'park/7'})

    assert check_splits(dataset) is None


def test_an_unreadable_run_json_is_an_error(tmp_path):
    dataset = make_dataset(tmp_path, {CLIP: ('indoor', 'train')})
    (dataset.runs_dir / 'r1').mkdir(parents=True)
    (dataset.runs_dir / 'r1' / 'run.json').write_text('{"clips": ')

    with pytest.raises(DatasetError, match='run.json'):
        check_splits(dataset)


def test_labeled_frames_skip_unlabeled_frames_and_other_splits(tmp_path):
    dataset = make_dataset(tmp_path, {
        'a': ('indoor', 'train'),
        'b': ('indoor', 'val'),
        'c': ('outdoor', 'train'),
    })
    a_labeled, a_unlabeled, a_empty = frame_row('a', 0), frame_row('a', 10), frame_row('a', 20)
    b_labeled = frame_row('b', 0)
    c_labeled = frame_row('c', 0, domain='outdoor')
    dataset.write_frames([a_labeled, a_unlabeled, a_empty, b_labeled, c_labeled])
    for row in (a_labeled, b_labeled, c_labeled):
        write_label(dataset, row)
    write_label(dataset, a_empty, text='')  # labeled, with no branch in view

    assert labeled_frames(dataset, 'train') == [a_labeled, a_empty, c_labeled]
    assert labeled_frames(dataset, 'train', domain='indoor') == [a_labeled, a_empty]
    assert labeled_frames(dataset, 'val') == [b_labeled]
    assert not dataset.is_labeled(a_unlabeled.frame_id)


def test_the_data_yaml_points_at_lists_of_image_paths(tmp_path):
    dataset = Dataset(tmp_path / 'vespr_dataset')
    train = [frame_row(CLIP, 0), frame_row(CLIP, 10)]
    val = [frame_row(OTHER_CLIP, 0)]
    out_dir = tmp_path / 'run'

    yaml_path = write_data_yaml(dataset, out_dir, train, val)

    data = yaml.safe_load(yaml_path.read_text())
    assert yaml_path == out_dir / 'data.yaml'
    assert data['names'] == {0: 'branch'}
    assert Path(data['path']) / data['train'] == out_dir / 'train.txt'
    assert Path(data['path']) / data['val'] == out_dir / 'val.txt'
    train_lines = (out_dir / 'train.txt').read_text().splitlines()
    assert train_lines == [str(dataset.image_path(row.frame_id)) for row in train]
    for line in train_lines:
        assert Path(line).is_absolute() and Path(line).parent == dataset.images_dir
    val_lines = (out_dir / 'val.txt').read_text().splitlines()
    assert val_lines == [str(dataset.image_path(val[0].frame_id))]
