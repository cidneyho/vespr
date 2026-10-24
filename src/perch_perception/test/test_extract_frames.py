"""Tests for training/extract_frames.py, which turns .bag clips into PNG frames and label batches.

Most tests use fake .bag files (a few bytes each, so their size and MD5 are real) and an open_bag
that returns an in-memory ListSource, so they need no camera and no pyrealsense2. The last test
replays a real synthetic .bag.
"""

import hashlib
import json
import zipfile
from pathlib import Path

import numpy as np
import pytest

# extract_frames writes PNGs with OpenCV, so the whole file is skipped without it.
cv2 = pytest.importorskip('cv2')

from fake_source import ListSource, make_frame  # noqa: E402
from perch_perception.training.dataset import (  # noqa: E402
    ClipRow, Dataset, DatasetError, frame_id)
from perch_perception.training.extract_frames import (  # noqa: E402
    extract_clip, extract_folder, main, pack_batches)


def numbered_frames(count):
    """Frames whose image is filled with the frame's index, so a PNG shows which frame it is."""
    return [make_frame(i / 30)._replace(image=np.full((2, 4), i, dtype=np.uint8))
            for i in range(count)]


def write_bag(folder, clip, content=None):
    """Write a fake .bag file. Only its bytes matter, for the size and MD5 in clips.csv."""
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f'{clip}.bag'
    path.write_bytes(content if content is not None else f'recording {clip}'.encode())
    return path


def write_details(folder, clip, **details):
    """Write the .json the recorder saves next to a clip's .bag."""
    path = folder / f'{clip}.json'
    path.write_text(json.dumps({'clip': clip, 'bad': False, **details}))
    return path


def open_fake_bags(frame_counts):
    """Return an open_bag that gives frame_counts[clip] numbered frames for each clip."""
    def open_bag(path):
        return ListSource(numbered_frames(frame_counts[path.stem]))
    return open_bag


def zip_names(path):
    with zipfile.ZipFile(path) as archive:
        return archive.namelist()


def test_every_tenth_frame_is_kept_starting_with_the_first(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')

    rows = extract_clip(dataset, ListSource(numbered_frames(25)), 'a', 'indoor', every=10)

    assert [row.frame_index for row in rows] == [0, 10, 20]
    assert [row.frame_id for row in rows] == ['a_000000', 'a_000010', 'a_000020']
    assert all(row.clip == 'a' and row.domain == 'indoor' and row.batch == '' for row in rows)
    assert rows[1].timestamp == pytest.approx(10 / 30)
    image = cv2.imread(str(dataset.image_path(frame_id('a', 10))), cv2.IMREAD_UNCHANGED)
    assert image.ndim == 2
    assert image.dtype == np.uint8
    assert (image == 10).all()


def test_extraction_records_clips_with_size_and_md5(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')
    content = b'the bytes of clip a'
    write_bag(tmp_path / 'bags', 'a', content)

    extract_folder(dataset, tmp_path / 'bags', 'indoor', open_bag=open_fake_bags({'a': 25}))

    assert dataset.read_clips() == [
        ClipRow('a', 'indoor', len(content), hashlib.md5(content).hexdigest())]
    assert [row.frame_id for row in dataset.read_frames()] == [
        'a_000000', 'a_000010', 'a_000020']


def test_a_second_run_extracts_only_new_clips(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    open_bag = open_fake_bags({'a': 25, 'b': 15})
    write_bag(folder, 'a')
    extract_folder(dataset, folder, 'indoor', open_bag=open_bag)
    first_batch = dataset.to_label_dir / 'batch_001.zip'
    first_batch_bytes = first_batch.read_bytes()

    write_bag(folder, 'b')
    summary = extract_folder(dataset, folder, 'indoor', open_bag=open_bag)

    assert summary.skipped == ['a']
    assert summary.extracted == ['b']
    assert summary.frames == 2
    assert summary.batches == [dataset.to_label_dir / 'batch_002.zip']
    assert first_batch.read_bytes() == first_batch_bytes
    batches = {row.frame_id: row.batch for row in dataset.read_frames()}
    assert batches == {
        'a_000000': 'batch_001', 'a_000010': 'batch_001', 'a_000020': 'batch_001',
        'b_000000': 'batch_002', 'b_000010': 'batch_002',
    }
    assert sorted(zip_names(summary.batches[0])) == ['b_000000.png', 'b_000010.png']


def test_a_clip_with_a_known_name_but_different_content_is_refused(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    open_bag = open_fake_bags({'a': 25})
    write_bag(folder, 'a', b'first recording')
    extract_folder(dataset, folder, 'indoor', open_bag=open_bag)
    frames_before = dataset.read_frames()

    write_bag(folder, 'a', b'a different recording with the same name')
    with pytest.raises(DatasetError, match='a.bag'):
        extract_folder(dataset, folder, 'indoor', open_bag=open_bag)

    assert dataset.read_frames() == frames_before
    assert len(dataset.read_clips()) == 1


def test_held_clips_are_written_to_splits_as_hold(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    write_bag(folder, 'a')
    write_bag(folder, 'b')

    extract_folder(dataset, folder, 'outdoor', hold=['b'],
                   open_bag=open_fake_bags({'a': 25, 'b': 25}))

    assert dataset.read_splits() == {'b': 'hold'}


def test_holding_a_clip_that_is_not_in_the_folder_is_an_error(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    write_bag(folder, 'a')

    with pytest.raises(DatasetError, match='missing'):
        extract_folder(dataset, folder, 'indoor', hold=['missing'],
                       open_bag=open_fake_bags({'a': 25}))

    assert dataset.read_frames() == []
    assert dataset.read_clips() == []


def test_holding_a_clip_that_was_already_extracted_is_an_error(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    open_bag = open_fake_bags({'a': 25, 'b': 25})
    write_bag(folder, 'a')
    extract_folder(dataset, folder, 'indoor', open_bag=open_bag)
    write_bag(folder, 'b')

    # The clip is not labeled, so it has no row in splits.csv yet: the message says add or change.
    with pytest.raises(DatasetError, match='Add or change its row in splits.csv'):
        extract_folder(dataset, folder, 'indoor', hold=['a'], open_bag=open_bag)

    assert dataset.read_splits() == {}
    assert [row.clip for row in dataset.read_clips()] == ['a']


def test_a_clip_marked_bad_is_not_extracted(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    write_bag(folder, 'a')
    write_bag(folder, 'b')
    write_details(folder, 'a', bad=True)

    summary = extract_folder(dataset, folder, 'outdoor',
                             open_bag=open_fake_bags({'a': 25, 'b': 25}))

    assert summary.bad == ['a']
    assert summary.extracted == ['b']
    assert [row.clip for row in dataset.read_clips()] == ['b']
    assert {row.clip for row in dataset.read_frames()} == {'b'}


def test_a_clip_marked_bad_after_it_was_extracted_is_reported_and_kept(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    open_bag = open_fake_bags({'a': 25})
    write_bag(folder, 'a')
    extract_folder(dataset, folder, 'outdoor', open_bag=open_bag)
    write_details(folder, 'a', bad=True)

    summary = extract_folder(dataset, folder, 'outdoor', open_bag=open_bag)

    assert summary.bad_extracted == ['a']
    assert [row.clip for row in dataset.read_clips()] == ['a']


def test_bad_clips_are_listed_by_main(tmp_path, capsys, monkeypatch):
    folder = tmp_path / 'bags'
    write_bag(folder, 'a')
    write_details(folder, 'a', bad=True)

    main([str(folder), '--domain', 'outdoor', '--dataset', str(tmp_path / 'dataset')])

    assert 'skipped 1 clips marked bad: a' in capsys.readouterr().out


@pytest.mark.parametrize('site, branch, group', [
    (' Big  Oak ', '7', 'big oak/7'),  # trimmed, spaces squeezed, lower case
    ('', 'B7', 'b7'),
    ('park', '', ''),  # a site alone names no branch
])
def test_the_branch_in_the_details_goes_into_clips_csv(tmp_path, site, branch, group):
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    write_bag(folder, 'a')
    write_details(folder, 'a', domain='outdoor', site=site, branch=branch)

    extract_folder(dataset, folder, 'outdoor', open_bag=open_fake_bags({'a': 25}))

    assert dataset.read_clips()[0].group == group


def test_a_clip_without_details_has_no_branch(tmp_path):
    # Clips recorded before the recorder wrote a .json have only the .bag and .mp4.
    dataset = Dataset(tmp_path / 'dataset')
    write_bag(tmp_path / 'bags', 'a')

    extract_folder(dataset, tmp_path / 'bags', 'outdoor', open_bag=open_fake_bags({'a': 25}))

    assert dataset.read_clips()[0].group == ''


@pytest.mark.parametrize('text', ['{"bad": tru', '["not", "an", "object"]', '{"bad": "yes"}',
                                  '{"branch": 7}'])
def test_a_details_file_that_cannot_be_read_is_an_error_naming_it(tmp_path, text):
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    write_bag(folder, 'a')
    (folder / 'a.json').write_text(text)

    with pytest.raises(DatasetError, match='a.json'):
        extract_folder(dataset, folder, 'outdoor', open_bag=open_fake_bags({'a': 25}))

    assert dataset.read_clips() == []


def test_a_clip_recorded_for_the_other_domain_is_refused(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    write_bag(folder, 'a')
    write_details(folder, 'a', domain='indoor')

    with pytest.raises(DatasetError, match='indoor'):
        extract_folder(dataset, folder, 'outdoor', open_bag=open_fake_bags({'a': 25}))

    assert dataset.read_clips() == []


def test_holding_a_clip_marked_bad_is_an_error(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    write_bag(folder, 'a')
    write_details(folder, 'a', bad=True)

    with pytest.raises(DatasetError, match='marked bad'):
        extract_folder(dataset, folder, 'outdoor', hold=['a'],
                       open_bag=open_fake_bags({'a': 25}))


def test_holding_a_clip_whose_branch_already_has_another_split_is_an_error(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    open_bag = open_fake_bags({'a': 25, 'b': 25})
    write_bag(folder, 'a')
    write_details(folder, 'a', site='park', branch='7')
    extract_folder(dataset, folder, 'outdoor', open_bag=open_bag)
    dataset.write_splits({'a': 'train'})  # as an import would, once a is labeled
    write_bag(folder, 'b')
    write_details(folder, 'b', site='park', branch='7')

    with pytest.raises(DatasetError, match='park/7'):
        extract_folder(dataset, folder, 'outdoor', hold=['b'], open_bag=open_bag)

    assert [row.clip for row in dataset.read_clips()] == ['a']


def test_holding_one_clip_of_a_new_branch_is_allowed(tmp_path):
    # The branch's other clips join hold when their labels are imported.
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    for clip in ('a', 'b'):
        write_bag(folder, clip)
        write_details(folder, clip, site='park', branch='7')

    extract_folder(dataset, folder, 'outdoor', hold=['a'],
                   open_bag=open_fake_bags({'a': 25, 'b': 25}))

    assert dataset.read_splits() == {'a': 'hold'}


def test_a_bag_that_fails_partway_leaves_earlier_clips_and_no_rows_for_itself(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    write_bag(folder, 'a')
    write_bag(folder, 'b')

    def five_frames_then_an_error():
        yield from numbered_frames(5)
        raise RuntimeError('Frame didn\'t arrive within 1000')

    def open_bag(path):
        if path.stem == 'b':
            return ListSource(five_frames_then_an_error())
        return ListSource(numbered_frames(25))

    with pytest.raises(DatasetError, match='b.bag'):
        extract_folder(dataset, folder, 'indoor', open_bag=open_bag)

    assert {row.clip for row in dataset.read_frames()} == {'a'}
    assert [row.clip for row in dataset.read_clips()] == ['a']


def test_a_clip_left_in_the_manifest_by_a_crash_is_extracted_again_without_duplicates(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    open_bag = open_fake_bags({'a': 25, 'b': 25})
    write_bag(folder, 'a')
    extract_folder(dataset, folder, 'indoor', open_bag=open_bag)
    # A crash between writing manifest.csv and clips.csv leaves b's frames with no clips.csv row.
    write_bag(folder, 'b')
    rows = extract_clip(dataset, open_bag(folder / 'b.bag'), 'b', 'indoor', every=10)
    dataset.write_frames(dataset.read_frames() + rows)

    extract_folder(dataset, folder, 'indoor', open_bag=open_bag)

    assert [row.frame_id for row in dataset.read_frames() if row.clip == 'b'] == [
        frame_id('b', 0), frame_id('b', 10), frame_id('b', 20)]
    assert [row.clip for row in dataset.read_clips()] == ['a', 'b']


def test_batches_hold_at_most_batch_size_frames(tmp_path):
    dataset = Dataset(tmp_path / 'dataset')
    rows = extract_clip(dataset, ListSource(numbered_frames(250)), 'a', 'indoor', every=1)
    dataset.write_frames(rows)

    batches = pack_batches(dataset, batch_size=100)

    assert [path.name for path in batches] == [
        'batch_001.zip', 'batch_002.zip', 'batch_003.zip']
    assert [len(zip_names(path)) for path in batches] == [100, 100, 50]
    # The PNGs sit at the top of each zip, with no folders, named after their frame.
    assert zip_names(batches[0])[0] == 'a_000000.png'
    assert all('/' not in name for path in batches for name in zip_names(path))
    assert [row.batch for row in dataset.read_frames()] == (
        ['batch_001'] * 100 + ['batch_002'] * 100 + ['batch_003'] * 50)


def test_extracting_the_synthetic_bag_end_to_end(tmp_path):
    pytest.importorskip('pyrealsense2')
    import synthetic_bag

    dataset = Dataset(tmp_path / 'dataset')
    folder = tmp_path / 'bags'
    folder.mkdir()
    synthetic_bag.write_synthetic_bag(folder / 'synthetic.bag')

    summary = extract_folder(dataset, folder, 'indoor', every=10)

    assert summary.extracted == ['synthetic']
    assert summary.frames == synthetic_bag.FRAME_COUNT // 10
    assert [row.frame_index for row in dataset.read_frames()] == [0, 10, 20, 30]
    image = cv2.imread(str(dataset.image_path(frame_id('synthetic', 10))), cv2.IMREAD_UNCHANGED)
    assert image.shape == (synthetic_bag.HEIGHT, synthetic_bag.WIDTH)
    assert (image == 10).all()
    assert summary.batches == [dataset.to_label_dir / 'batch_001.zip']


def test_the_details_the_recorder_writes_are_read_by_extraction(tmp_path):
    # Pins the .json format the two sides share: the recorder writes it, extraction reads it.
    from datetime import datetime

    from perch_perception.data_collection import record

    class Camera(ListSource):
        fps = 10

    def open_camera(record_to):
        if record_to is not None:
            write_bag(tmp_path / 'bags', Path(record_to).stem)  # stands in for librealsense
        return Camera([make_frame(0.0, shape=(48, 64))])

    folder = tmp_path / 'bags'
    recorder = record.Recorder(open_camera, folder, tmp_path / 'next_clip_id',
                               clock=lambda: datetime(2026, 10, 23, 14, 2, 11), domain='outdoor')
    for branch in ('7', '8'):
        recorder.start({'site': 'Park', 'branch': branch})
        recorder.run_session()
    recorder.mark_last_clip_bad(True)
    dataset = Dataset(tmp_path / 'dataset')

    summary = extract_folder(dataset, folder, 'outdoor',
                             open_bag=lambda path: ListSource(numbered_frames(25)))

    assert summary.extracted == ['0001_2026-10-23T14-02-11']
    assert summary.bad == ['0002_2026-10-23T14-02-11']
    assert dataset.read_clips()[0].group == 'park/7'
