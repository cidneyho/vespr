"""Tests for data_collection/trim.py, which sets the part of a clip that extract_frames keeps.

Every test works on a folder of fake clips under tmp_path: a .bag of a few bytes and the .json
the recorder writes. Nothing is replayed.
"""

import json
import os

import numpy as np

import pytest

from perch_perception.data_collection import trim

CLIP = '0042_2026-10-23T14-02-11'


def write_clip(folder, clip=CLIP, with_json=True, **details):
    """Write a fake clip: its .bag and, if with_json, the .json the recorder writes."""
    folder.mkdir(parents=True, exist_ok=True)
    (folder / f'{clip}.bag').write_bytes(b'recording')
    if with_json:
        (folder / f'{clip}.json').write_text(json.dumps(
            {'clip': clip, 'number': 42, 'seconds': 14.2, 'bad': True, **details}))


def read_details(folder, clip=CLIP):
    return json.loads((folder / f'{clip}.json').read_text())


def test_keeping_part_of_a_clip_saves_the_range_and_clears_bad(tmp_path):
    write_clip(tmp_path)

    trim.set_keep(trim.find_clip(tmp_path, CLIP), 0, 12.5)

    details = read_details(tmp_path)
    assert details['keep'] == [0, 12.5]
    assert details['bad'] is False
    assert details['number'] == 42  # the rest is left as it was


@pytest.mark.parametrize('name', ['42', '0042', CLIP, CLIP + '.bag'])
def test_a_clip_is_found_by_its_number_or_its_name(tmp_path, name):
    write_clip(tmp_path)
    write_clip(tmp_path, '0043_2026-10-23T14-03-40')

    assert trim.find_clip(tmp_path, name) == tmp_path / f'{CLIP}.bag'


def test_a_number_two_clips_share_is_refused(tmp_path):
    # Numbers start again if the counter file is lost, so ask for the whole name then.
    write_clip(tmp_path)
    write_clip(tmp_path, '0042_2026-06-05T09-00-00')

    with pytest.raises(trim.TrimError, match='0042_2026-06-05T09-00-00'):
        trim.find_clip(tmp_path, '42')


def test_a_clip_that_is_not_in_the_folder_is_refused(tmp_path):
    write_clip(tmp_path)

    with pytest.raises(trim.TrimError, match='43'):
        trim.find_clip(tmp_path, '43')


@pytest.mark.parametrize('start, end', [(5, 5), (8, 3), (-1, 4), (float('nan'), 4),
                                        (0, float('inf'))])
def test_a_range_that_is_not_from_start_up_to_a_later_end_is_refused(tmp_path, start, end):
    write_clip(tmp_path)

    with pytest.raises(trim.TrimError):
        trim.set_keep(tmp_path / f'{CLIP}.bag', start, end)
    assert 'keep' not in read_details(tmp_path)


def test_a_range_starting_after_the_clip_ends_is_refused(tmp_path):
    write_clip(tmp_path)  # 14.2 s long

    with pytest.raises(trim.TrimError, match='14.2'):
        trim.set_keep(tmp_path / f'{CLIP}.bag', 15, 20)


def test_an_end_past_the_end_of_the_clip_keeps_the_rest_of_it(tmp_path):
    write_clip(tmp_path)

    trim.set_keep(tmp_path / f'{CLIP}.bag', 3, 99)

    assert read_details(tmp_path)['keep'] == [3, 99]


def test_keeping_all_of_a_clip_removes_the_range(tmp_path):
    write_clip(tmp_path)
    trim.set_keep(tmp_path / f'{CLIP}.bag', 0, 12.5)

    trim.set_keep(tmp_path / f'{CLIP}.bag', None, None)

    assert 'keep' not in read_details(tmp_path)


def test_a_clip_from_before_the_details_file_gets_one(tmp_path):
    old = '2026-10-15T14-02-11'
    write_clip(tmp_path, old, with_json=False)

    trim.set_keep(trim.find_clip(tmp_path, old), 2, 9)

    assert read_details(tmp_path, old) == {'keep': [2, 9], 'bad': False}


def test_a_details_file_that_is_not_json_is_refused_and_left_alone(tmp_path):
    write_clip(tmp_path)
    (tmp_path / f'{CLIP}.json').write_text('{"bad": tru')

    with pytest.raises(trim.TrimError, match=f'{CLIP}.json'):
        trim.set_keep(tmp_path / f'{CLIP}.bag', 0, 5)
    assert (tmp_path / f'{CLIP}.json').read_text() == '{"bad": tru'


def test_main_keeps_part_of_a_clip_and_says_what_it_did(tmp_path, capsys):
    write_clip(tmp_path)

    status = trim.main(['42', '--keep', '0', '12.5', '--domain', 'outdoor',
                        '--folder', str(tmp_path)])

    assert status == 0
    out = capsys.readouterr().out
    assert CLIP in out and '0 to 12.5 s of 14.2 s' in out and 'no longer marked bad' in out


def test_main_with_all_keeps_the_whole_clip(tmp_path, capsys):
    write_clip(tmp_path, keep=[0, 12.5], bad=False)

    status = trim.main(['42', '--all', '--domain', 'outdoor', '--folder', str(tmp_path)])

    assert status == 0
    assert 'keep' not in read_details(tmp_path)
    assert 'whole clip' in capsys.readouterr().out


def test_main_reports_an_error_without_a_traceback(tmp_path, capsys):
    write_clip(tmp_path)

    status = trim.main(['43', '--all', '--domain', 'outdoor', '--folder', str(tmp_path)])

    assert status == 1
    assert 'error:' in capsys.readouterr().err


def test_keep_and_all_cannot_be_given_together(tmp_path):
    with pytest.raises(SystemExit):
        trim.parse_args(['42', '--keep', '0', '5', '--all', '--domain', 'outdoor'])


def test_the_domain_picks_the_clip_folder(monkeypatch):
    monkeypatch.delenv('SUDO_USER', raising=False)

    args = trim.parse_args(['42', '--all', '--domain', 'indoor'])

    assert args.folder == os.path.expanduser('~/vespr_recordings/indoor')


def test_extraction_reads_the_range_trim_writes(tmp_path):
    # Pins the format the two sides share.
    from perch_perception.training.extract_frames import read_details as extract_reads

    write_clip(tmp_path)
    trim.set_keep(tmp_path / f'{CLIP}.bag', 1.5, 9)

    assert extract_reads(tmp_path / f'{CLIP}.bag')['keep'] == [1.5, 9]


def test_minus_zero_is_saved_as_zero(tmp_path):
    write_clip(tmp_path)

    trim.set_keep(tmp_path / f'{CLIP}.bag', -0.0, 5)

    assert str(read_details(tmp_path)['keep'][0]) == '0.0'


def test_trimming_a_clip_that_did_not_finish_warns_that_its_bag_may_not_open(tmp_path, capsys):
    # librealsense cannot open a truncated .bag, and trimming does not change that.
    write_clip(tmp_path, error='the recording did not finish', seconds=None)

    trim.main(['42', '--keep', '0', '5', '--domain', 'outdoor', '--folder', str(tmp_path)])

    assert 'may not open' in capsys.readouterr().out


def test_a_preview_video_shorter_than_the_clip_is_warned_about(tmp_path, capsys):
    # Frames the recorder dropped are missing from the .mp4, so its times run behind the clip's.
    from perch_perception.data_collection.record import PreviewVideo

    cv2 = pytest.importorskip('cv2')
    write_clip(tmp_path, seconds=14.2)
    video = PreviewVideo(str(tmp_path / f'{CLIP}.mp4'), 10)
    for _ in range(100):  # 10 s at 10 fps
        video.write(np.zeros((48, 64), dtype=np.uint8))
    video.close()
    assert cv2.VideoCapture(str(tmp_path / f'{CLIP}.mp4')).isOpened()

    trim.main(['42', '--keep', '0', '5', '--domain', 'outdoor', '--folder', str(tmp_path)])

    out = capsys.readouterr().out
    assert 'warning' in out and '10 s' in out and '14.2 s' in out
