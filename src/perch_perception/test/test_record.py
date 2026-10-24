"""Tests for record.py: clip numbers and names, clip details, the sessions, and the web page.

No camera is used. FakeCamera stands in for the function that opens the camera
and hands out in-memory sources.
"""

import json
import os
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from fake_source import ListSource, make_frame  # noqa: E402
from perch_perception.data_collection import record  # noqa: E402

NOW = datetime(2026, 10, 10, 14, 32, 5)
CLIP = '0001_2026-10-10T14-32-05'  # the first clip a fresh counter hands out
SHAPE = (48, 64)  # rows, columns: big enough for a video encoder


class ScriptedSource(ListSource):
    """A source that can run a callback just before it yields a chosen frame."""

    fps = 10

    def __init__(self, frames, before_frame=None, callback=None):
        super().__init__(frames)
        self.before_frame = before_frame
        self.callback = callback
        self.yielded = 0

    def frames(self):
        for index, frame in enumerate(self._frames):
            if index == self.before_frame:
                self.callback()
            self.yielded += 1
            yield frame


class FakeCamera:
    """Stands in for the function that opens the camera. Recording creates an empty .bag."""

    def __init__(self, frame_count=5, before_frame=None, callback=None):
        self.frame_count = frame_count
        self.before_frame = before_frame
        self.callback = callback
        self.opened_with = []  # the record_to argument of every open
        self.sources = []

    def __call__(self, record_to):
        self.opened_with.append(record_to)
        if record_to is not None:
            open(record_to, 'wb').close()  # librealsense creates the .bag as the camera starts
        frames = [make_frame(float(i), shape=SHAPE) for i in range(self.frame_count)]
        source = ScriptedSource(frames, self.before_frame, self.callback)
        self.sources.append(source)
        return source


def make_recorder(camera, folder, counter=None, domain='outdoor'):
    # The real counter sits next to the domain folders; a test keeps its own in the clip folder.
    if counter is None:
        counter = os.path.join(str(folder), 'next_clip_id')
    return record.Recorder(camera, folder, counter, clock=lambda: NOW, domain=domain)


def record_one_clip(recorder, details=None):
    recorder.start(details)
    recorder.run_session()


def read_details(folder, clip=CLIP):
    return json.loads((folder / (clip + '.json')).read_text())


def count_video_frames(path):
    capture = cv2.VideoCapture(str(path))
    count = 0
    while capture.read()[0]:
        count += 1
    capture.release()
    return count


def test_clip_name_is_the_clip_number_then_the_start_time_without_colons():
    assert record.clip_name(42, NOW) == '0042_2026-10-10T14-32-05'


def test_clip_numbers_above_9999_keep_every_digit():
    assert record.clip_name(12345, NOW) == '12345_2026-10-10T14-32-05'


def test_without_a_counter_file_the_first_clip_is_number_1(tmp_path):
    counter = tmp_path / 'next_clip_id'

    assert record.take_clip_number(str(counter)) == 1
    assert counter.read_text().strip() == '2'


def test_clip_numbers_go_up_by_one(tmp_path):
    counter = str(tmp_path / 'next_clip_id')

    assert [record.take_clip_number(counter) for _ in range(3)] == [1, 2, 3]


def test_the_counter_carries_on_from_the_number_in_its_file(tmp_path):
    counter = tmp_path / 'next_clip_id'
    counter.write_text('120\n')

    assert record.take_clip_number(str(counter)) == 120


@pytest.mark.parametrize('text', ['twelve', '0', '-3', '1.5', ''])
def test_a_counter_file_that_is_not_a_number_from_1_is_refused(tmp_path, text):
    counter = tmp_path / 'next_clip_id'
    counter.write_text(text)

    with pytest.raises(RuntimeError, match='next_clip_id'):
        record.take_clip_number(str(counter))
    assert counter.read_text() == text


def test_the_counter_sits_next_to_the_domain_folders(monkeypatch):
    # One counter for indoor and outdoor, so a number names one clip only.
    monkeypatch.delenv('SUDO_USER', raising=False)

    assert record.counter_path() == os.path.expanduser('~/vespr_recordings/next_clip_id')


def test_indoor_and_outdoor_recorders_share_one_counter(tmp_path):
    counter = str(tmp_path / 'next_clip_id')
    indoor = make_recorder(FakeCamera(), tmp_path / 'indoor', counter, domain='indoor')
    outdoor = make_recorder(FakeCamera(), tmp_path / 'outdoor', counter, domain='outdoor')

    record_one_clip(indoor)
    record_one_clip(outdoor)

    assert indoor.status()['last_clip'] == '0001_2026-10-10T14-32-05'
    assert outdoor.status()['last_clip'] == '0002_2026-10-10T14-32-05'


def test_a_number_is_never_reused_when_its_clip_is_deleted(tmp_path):
    # clip_storage cleanup deletes uploaded clips from the Jetson, so the folder cannot say
    # which numbers were used.
    recorder = make_recorder(FakeCamera(), tmp_path)
    record_one_clip(recorder)
    for suffix in ('.bag', '.mp4', '.json'):
        (tmp_path / (CLIP + suffix)).unlink(missing_ok=True)

    record_one_clip(recorder)

    assert recorder.status()['last_clip'] == '0002_2026-10-10T14-32-05'


def test_a_broken_counter_stops_the_recording_before_the_camera_records(tmp_path):
    (tmp_path / 'next_clip_id').write_text('twelve')
    camera = FakeCamera()
    recorder = make_recorder(camera, tmp_path)

    record_one_clip(recorder)

    assert 'next_clip_id' in recorder.status()['error']
    assert recorder.status()['state'] == 'preview'
    assert camera.opened_with == []


def test_a_recording_writes_its_details_next_to_the_bag(tmp_path):
    record_one_clip(make_recorder(FakeCamera(), tmp_path))

    details = read_details(tmp_path)
    seconds = details.pop('seconds')
    assert details == {
        'clip': CLIP, 'number': 1, 'domain': 'outdoor', 'started': '2026-10-10T14:32:05',
        'site': '', 'branch': '', 'light': '', 'notes': '', 'bad': False}
    assert seconds >= 0


def test_while_recording_the_details_say_bad_until_the_clip_ends(tmp_path):
    # If the battery is pulled mid-clip, the .json on disk is what is left: it must say bad.
    seen = []
    camera = FakeCamera(before_frame=2, callback=lambda: seen.append(read_details(tmp_path)))

    record_one_clip(make_recorder(camera, tmp_path))

    assert seen[0]['bad'] is True
    assert seen[0]['error'] == 'the recording did not finish'
    assert seen[0]['seconds'] is None  # not known until the clip ends
    assert read_details(tmp_path)['bad'] is False
    assert 'error' not in read_details(tmp_path)


def test_a_failed_last_details_write_leaves_the_clip_bad_and_says_so(tmp_path, monkeypatch):
    real_write = record.write_details
    writes = []

    def write_then_fail(path, details):
        writes.append(path)
        if len(writes) > 1:
            raise OSError(28, 'No space left on device')
        real_write(path, details)

    monkeypatch.setattr(record, 'write_details', write_then_fail)
    recorder = make_recorder(FakeCamera(), tmp_path)

    record_one_clip(recorder)

    assert read_details(tmp_path)['bad'] is True
    assert recorder.status()['last_bad'] is True
    assert 'No space left' in recorder.status()['error']


def test_an_unexpected_error_during_a_clip_leaves_it_bad(tmp_path):
    def fail():
        raise ValueError('a bug in the frame loop')

    with pytest.raises(ValueError):
        record_one_clip(make_recorder(FakeCamera(before_frame=2, callback=fail), tmp_path))

    assert read_details(tmp_path)['bad'] is True


def test_ctrl_c_during_a_clip_saves_it_as_good_then_quits(tmp_path):
    # Ctrl+C is how the recorder is quit. The camera still closes, which finishes the .bag.
    def interrupt():
        raise KeyboardInterrupt

    camera = FakeCamera(before_frame=2, callback=interrupt)

    with pytest.raises(KeyboardInterrupt):
        record_one_clip(make_recorder(camera, tmp_path))

    assert camera.sources[0].closed
    assert read_details(tmp_path)['bad'] is False


def test_ctrl_c_while_the_camera_closes_leaves_the_clip_bad(tmp_path):
    # A second Ctrl+C while the camera closes can leave the .bag unfinished.
    class InterruptedClose(ScriptedSource):
        def close(self):
            raise KeyboardInterrupt

    def camera(record_to):
        open(record_to, 'wb').close()
        return InterruptedClose([make_frame(0.0, shape=SHAPE)] * 3)

    with pytest.raises(KeyboardInterrupt):
        record_one_clip(make_recorder(camera, tmp_path))

    assert read_details(tmp_path)['bad'] is True


def test_seconds_counts_to_the_last_frame_not_to_the_end_of_the_camera_shutdown(tmp_path):
    class SlowToClose(ScriptedSource):
        def close(self):
            time.sleep(0.5)
            super().close()

    def camera(record_to):
        return SlowToClose([make_frame(0.0, shape=SHAPE)] * 3)

    record_one_clip(make_recorder(camera, tmp_path))

    assert read_details(tmp_path)['seconds'] < 0.4


def test_seconds_counts_to_the_last_frame_after_a_camera_error_too(tmp_path):
    class FailsThenClosesSlowly(ScriptedSource):
        def close(self):
            time.sleep(0.5)
            super().close()

    def fail():
        raise RuntimeError('frame did not arrive within 5000')

    def camera(record_to):
        open(record_to, 'wb').close()
        return FailsThenClosesSlowly([make_frame(0.0, shape=SHAPE)] * 3, 2, fail)

    record_one_clip(make_recorder(camera, tmp_path))

    assert read_details(tmp_path)['seconds'] < 0.4


def test_unmarking_after_a_failed_last_write_saves_the_whole_clip(tmp_path, monkeypatch):
    real_write = record.write_details
    writes = []

    def fail_the_second_write(path, details):
        writes.append(path)
        if len(writes) == 2:
            raise OSError(28, 'No space left on device')
        real_write(path, details)

    monkeypatch.setattr(record, 'write_details', fail_the_second_write)
    recorder = make_recorder(FakeCamera(), tmp_path)
    record_one_clip(recorder)

    recorder.mark_last_clip_bad(False)

    details = read_details(tmp_path)
    assert details['bad'] is False
    assert 'error' not in details


def test_the_details_given_with_start_are_saved_with_the_clip(tmp_path):
    recorder = make_recorder(FakeCamera(), tmp_path)

    record_one_clip(recorder, {'site': ' Park ', 'branch': '7', 'light': 'overcast',
                               'notes': 'head-on'})

    details = read_details(tmp_path)
    assert (details['site'], details['branch'], details['light'], details['notes']) == (
        'Park', '7', 'overcast', 'head-on')


def test_the_details_carry_over_to_the_next_clip(tmp_path):
    recorder = make_recorder(FakeCamera(), tmp_path)
    record_one_clip(recorder, {'site': 'park', 'branch': '7'})

    record_one_clip(recorder)

    details = read_details(tmp_path, '0002_2026-10-10T14-32-05')
    assert (details['site'], details['branch']) == ('park', '7')
    assert recorder.status()['details']['branch'] == '7'


def test_an_unknown_detail_is_refused(tmp_path):
    recorder = make_recorder(FakeCamera(), tmp_path)

    with pytest.raises(ValueError, match='colour'):
        recorder.start({'colour': 'green'})
    assert recorder.status()['state'] == 'preview'


def test_a_detail_that_is_not_text_is_refused(tmp_path):
    recorder = make_recorder(FakeCamera(), tmp_path)

    with pytest.raises(ValueError, match='branch'):
        recorder.start({'branch': 7})


def test_a_bag_left_by_a_camera_that_failed_to_start_keeps_its_details_marked_bad(tmp_path):
    def camera_that_fails_after_creating_the_bag(record_to):
        open(record_to, 'wb').close()
        raise RuntimeError('the camera stopped while starting')

    record_one_clip(make_recorder(camera_that_fails_after_creating_the_bag, tmp_path))

    assert read_details(tmp_path)['bad'] is True
    assert read_details(tmp_path)['error'] == 'the camera stopped while starting'


def test_no_details_file_is_written_when_the_camera_fails_to_open(tmp_path):
    def broken_camera(record_to):
        raise RuntimeError('no RealSense camera is connected')

    record_one_clip(make_recorder(broken_camera, tmp_path))

    assert list(tmp_path.glob('*.json')) == []


def test_a_camera_error_while_recording_marks_the_clip_bad(tmp_path):
    # A clip cut short may not replay to the end, and extract_frames skips clips marked bad.
    def fail():
        raise RuntimeError('frame did not arrive within 5000')

    recorder = make_recorder(FakeCamera(before_frame=2, callback=fail), tmp_path)

    record_one_clip(recorder)

    details = read_details(tmp_path)
    assert details['bad'] is True
    assert 'frame did not arrive' in details['error']
    assert recorder.status()['last_bad'] is True


def test_the_last_clip_can_be_marked_bad_and_unmarked(tmp_path):
    recorder = make_recorder(FakeCamera(), tmp_path)
    record_one_clip(recorder)

    recorder.mark_last_clip_bad(True)
    assert read_details(tmp_path)['bad'] is True
    assert recorder.status()['last_bad'] is True

    recorder.mark_last_clip_bad(False)
    assert read_details(tmp_path)['bad'] is False
    assert recorder.status()['last_bad'] is False


def test_marking_bad_before_any_clip_is_refused(tmp_path):
    recorder = make_recorder(FakeCamera(), tmp_path)

    with pytest.raises(record.CannotMark, match='no clip'):
        recorder.mark_last_clip_bad(True)


def test_marking_bad_while_a_clip_is_recording_is_refused(tmp_path):
    refusals = []
    camera = FakeCamera(before_frame=2, callback=lambda: None)  # a first clip, to have one
    recorder = make_recorder(camera, tmp_path)
    record_one_clip(recorder)

    def try_to_mark():
        with pytest.raises(record.CannotMark, match='between clips') as error:
            recorder.mark_last_clip_bad(True)
        refusals.append(error)

    camera.callback = try_to_mark
    record_one_clip(recorder)

    assert len(refusals) == 1
    assert read_details(tmp_path)['bad'] is False


def test_marking_a_clip_that_was_already_cleaned_up_is_refused(tmp_path):
    # cleanup deletes uploaded clips; a lone .json would conflict with the Drive on every upload.
    def camera(record_to):
        if record_to is not None:
            open(record_to, 'wb').close()
        return ScriptedSource([make_frame(0.0, shape=SHAPE)])

    recorder = make_recorder(camera, tmp_path)
    record_one_clip(recorder)
    for path in tmp_path.glob(CLIP + '.*'):
        path.unlink()

    with pytest.raises(record.CannotMark, match='no longer'):
        recorder.mark_last_clip_bad(True)
    assert not (tmp_path / (CLIP + '.json')).exists()


def test_under_sudo_the_details_and_the_counter_are_handed_back(tmp_path, monkeypatch):
    handed = []
    monkeypatch.setattr(record.os, 'chown', lambda path, uid, gid: handed.append(str(path)))
    monkeypatch.setenv('SUDO_UID', '501')
    monkeypatch.setenv('SUDO_GID', '20')
    counter = tmp_path / 'recordings' / 'next_clip_id'

    record_one_clip(make_recorder(FakeCamera(), tmp_path / 'recordings' / 'outdoor', counter))

    assert str(counter) in handed
    assert str(counter.parent) in handed  # the folder the counter made
    assert str(tmp_path / 'recordings' / 'outdoor' / (CLIP + '.json')) in handed


def test_default_folder_is_in_the_home_folder_outside_icloud(monkeypatch):
    # Desktop and Documents can be synced to iCloud Drive, and clips are
    # hundreds of megabytes, so the default sits directly in the home folder.
    monkeypatch.delenv('SUDO_USER', raising=False)

    assert record.default_folder() == os.path.expanduser('~/vespr_recordings')


def test_under_sudo_files_are_handed_back_to_the_user(tmp_path, monkeypatch):
    # Only root may give a file to another user, so the chown call itself is
    # recorded here instead of being carried out.
    calls = []
    monkeypatch.setattr(record.os, 'chown', lambda *args: calls.append(args))
    monkeypatch.setenv('SUDO_UID', '501')
    monkeypatch.setenv('SUDO_GID', '20')

    record.hand_to_user(str(tmp_path))

    assert calls == [(str(tmp_path), 501, 20)]


def test_without_sudo_file_ownership_is_left_alone(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(record.os, 'chown', lambda *args: calls.append(args))
    monkeypatch.delenv('SUDO_UID', raising=False)

    record.hand_to_user(str(tmp_path))

    assert calls == []


def test_a_preview_session_does_not_record(tmp_path):
    camera = FakeCamera()
    recorder = make_recorder(camera, tmp_path)

    recorder.run_session()

    assert camera.opened_with == [None]
    assert list(tmp_path.iterdir()) == []


def test_a_preview_session_publishes_the_camera_image_as_jpeg(tmp_path):
    recorder = make_recorder(FakeCamera(), tmp_path)

    recorder.run_session()

    _, jpeg = recorder.latest_jpeg()
    image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    assert image.shape == SHAPE


def test_a_preview_session_ends_when_recording_is_requested(tmp_path):
    camera = FakeCamera(frame_count=10, before_frame=3)
    recorder = make_recorder(camera, tmp_path)
    camera.callback = recorder.start

    recorder.run_session()

    assert camera.sources[0].yielded < 10
    assert camera.sources[0].closed


def test_a_recording_session_asks_the_camera_to_record_to_a_bag(tmp_path):
    camera = FakeCamera()
    recorder = make_recorder(camera, tmp_path)
    recorder.start()

    recorder.run_session()

    assert camera.opened_with == [str(tmp_path / (CLIP + '.bag'))]


def test_a_recording_session_writes_every_frame_to_the_preview_video(tmp_path):
    recorder = make_recorder(FakeCamera(frame_count=5), tmp_path)
    recorder.start()

    recorder.run_session()

    assert count_video_frames(tmp_path / (CLIP + '.mp4')) == 5


def test_a_recording_session_ends_when_stop_is_requested(tmp_path):
    camera = FakeCamera(frame_count=10, before_frame=3)
    recorder = make_recorder(camera, tmp_path)
    camera.callback = recorder.stop
    recorder.start()

    recorder.run_session()

    assert camera.sources[0].yielded < 10
    assert camera.sources[0].closed


def test_status_reports_the_clip_while_recording(tmp_path):
    seen = []
    camera = FakeCamera(before_frame=2)
    recorder = make_recorder(camera, tmp_path)
    camera.callback = lambda: seen.append(recorder.status())
    recorder.start()

    recorder.run_session()

    assert seen[0]['state'] == 'recording'
    assert seen[0]['clip'] == CLIP


def test_after_a_recording_the_recorder_is_back_in_preview(tmp_path):
    recorder = make_recorder(FakeCamera(), tmp_path)
    recorder.start()

    recorder.run_session()

    status = recorder.status()
    assert status['state'] == 'preview'
    assert status['last_clip'] == CLIP


def test_status_says_starting_between_the_request_and_the_first_frame(tmp_path):
    recorder = make_recorder(FakeCamera(), tmp_path)

    recorder.start()

    assert recorder.status()['state'] == 'starting'


def test_the_recordings_folder_is_created_when_missing(tmp_path):
    folder = tmp_path / 'new' / 'recordings'
    recorder = make_recorder(FakeCamera(), folder)
    recorder.start()

    recorder.run_session()

    assert (folder / (CLIP + '.mp4')).exists()


def test_a_camera_that_fails_to_open_is_reported_and_recording_is_cancelled(tmp_path):
    def broken_camera(record_to):
        raise RuntimeError("Couldn't resolve requests")

    recorder = make_recorder(broken_camera, tmp_path)
    recorder.start()

    recorder.run_session()

    status = recorder.status()
    assert "Couldn't resolve requests" in status['error']
    assert status['state'] == 'preview'


def test_a_repeating_camera_error_is_printed_once(tmp_path, capsys):
    def broken_camera(record_to):
        raise RuntimeError('no RealSense camera is connected')

    recorder = make_recorder(broken_camera, tmp_path)

    recorder.run_session()
    recorder.run_session()

    assert capsys.readouterr().out.count('no RealSense camera is connected') == 1


class EndlessSource(ListSource):
    """Yields the same frame at about 100 fps until it is closed, like a camera."""

    fps = 10

    def frames(self):
        while not self.closed:
            time.sleep(0.01)
            yield self._frames[0]


def wait_for(condition, seconds=5):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.01)
    return False


def test_the_camera_loop_follows_the_button(tmp_path):
    opened_with = []

    def camera(record_to):
        opened_with.append(record_to)
        return EndlessSource([make_frame(0.0, shape=SHAPE)])

    recorder = make_recorder(camera, tmp_path)
    loop = threading.Thread(target=recorder.run)
    loop.start()
    try:
        recorder.start()
        assert wait_for(lambda: recorder.status()['state'] == 'recording')
        recorder.stop()
        assert wait_for(lambda: recorder.status()['last_clip'] == CLIP)
        assert wait_for(lambda: len(opened_with) == 3)
    finally:
        recorder.quit()
        loop.join(timeout=5)

    # Preview, then the clip, then preview again.
    assert opened_with == [None, str(tmp_path / (CLIP + '.bag')), None]
    assert count_video_frames(tmp_path / (CLIP + '.mp4')) > 0
    assert not loop.is_alive()


@pytest.fixture
def served(tmp_path):
    """A recorder behind a running web server. Yields (recorder, base URL)."""
    recorder = make_recorder(FakeCamera(), tmp_path)
    server = record.make_server(recorder, port=0)  # port 0: any free port
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield recorder, f'http://127.0.0.1:{server.server_address[1]}'
    server.shutdown()
    server.server_close()


def post(url, body=None):
    data = b'' if body is None else json.dumps(body).encode()
    request = urllib.request.Request(url, data=data, method='POST',
                                     headers={'Content-Type': 'application/json'})
    try:
        return urllib.request.urlopen(request, timeout=5).status
    except urllib.error.HTTPError as error:
        return error.code


def test_the_page_has_a_record_button(served):
    _, base = served

    page = urllib.request.urlopen(base + '/', timeout=5).read().decode()

    assert '<button' in page
    assert 'Record' in page


def test_posting_start_requests_a_recording(served):
    recorder, base = served

    assert post(base + '/start') == 200
    assert recorder.status()['state'] == 'starting'


def test_posting_start_with_details_keeps_them_for_the_clip(served):
    recorder, base = served

    assert post(base + '/start', {'site': 'park', 'branch': '7'}) == 200
    assert recorder.status()['details']['branch'] == '7'


def test_posting_start_with_an_unknown_detail_is_refused(served):
    recorder, base = served

    assert post(base + '/start', {'colour': 'green'}) == 400
    assert recorder.status()['state'] == 'preview'


def test_posting_start_with_a_body_that_is_not_json_is_refused(served):
    recorder, base = served
    request = urllib.request.Request(base + '/start', data=b'{oops', method='POST')

    with pytest.raises(urllib.error.HTTPError) as error:
        urllib.request.urlopen(request, timeout=5)
    assert error.value.code == 400
    assert recorder.status()['state'] == 'preview'


def test_posting_bad_marks_the_last_clip(served, tmp_path):
    recorder, base = served
    record_one_clip(recorder)

    assert post(base + '/bad', {'bad': True}) == 200
    assert read_details(tmp_path)['bad'] is True


def test_posting_bad_before_any_clip_is_refused(served):
    _, base = served

    assert post(base + '/bad', {'bad': True}) == 409


@pytest.mark.parametrize('body', [{'bad': 'yes'}, {'bad': True, 'clip': '0007'}, [True]])
def test_posting_bad_with_any_other_body_is_refused(served, body):
    recorder, base = served
    record_one_clip(recorder)

    assert post(base + '/bad', body) == 400


def test_a_bad_mark_that_cannot_be_written_gets_an_error_reply(served, monkeypatch):
    recorder, base = served
    record_one_clip(recorder)

    def fail(path, details):
        raise OSError(28, 'No space left on device')

    monkeypatch.setattr(record, 'write_details', fail)

    assert post(base + '/bad', {'bad': True}) == 500
    assert recorder.status()['last_bad'] is False


def test_posting_stop_ignores_its_body(served):
    recorder, base = served
    recorder.start()
    request = urllib.request.Request(base + '/stop', data=b'{oops', method='POST')

    assert urllib.request.urlopen(request, timeout=5).status == 200
    assert recorder.status()['state'] == 'preview'


def test_the_page_has_the_detail_fields_and_the_bad_button(served):
    _, base = served

    page = urllib.request.urlopen(base + '/', timeout=5).read().decode()

    for field in ('site', 'branch', 'light', 'notes'):
        assert f'id="{field}"' in page
    assert 'id="bad"' in page


def test_posting_stop_cancels_the_request(served):
    recorder, base = served
    recorder.start()

    assert post(base + '/stop') == 200
    assert recorder.status()['state'] == 'preview'


def test_status_is_served_as_json(served):
    recorder, base = served

    body = urllib.request.urlopen(base + '/status', timeout=5).read()

    assert json.loads(body) == recorder.status()


def test_the_stream_serves_the_latest_camera_image(served):
    recorder, base = served
    recorder.run_session()  # a preview session, so there is an image to send

    response = urllib.request.urlopen(base + '/stream', timeout=5)
    start = response.read(200)
    response.close()

    assert response.headers.get_content_type() == 'multipart/x-mixed-replace'
    assert b'Content-Type: image/jpeg' in start


def test_the_domain_is_required():
    with pytest.raises(SystemExit):
        record.parse_args([])


def test_the_domain_picks_the_clip_folder(monkeypatch):
    monkeypatch.delenv('SUDO_USER', raising=False)

    args = record.parse_args(['--domain', 'outdoor'])

    assert args.out == os.path.expanduser('~/vespr_recordings/outdoor')


def test_out_replaces_the_domain_folder():
    args = record.parse_args(['--domain', 'indoor', '--out', '/clips'])

    assert args.out == '/clips'


def test_a_clip_never_replaces_an_earlier_clip_with_the_same_name(tmp_path):
    # Without internet the Jetson cannot set its clock, so a new clip can get
    # the start time, and so the name, of one recorded earlier.
    (tmp_path / (CLIP + '.bag')).write_bytes(b'earlier clip')
    (tmp_path / (CLIP + '-2.mp4')).write_bytes(b'earlier clip')
    camera = FakeCamera()
    recorder = make_recorder(camera, tmp_path)
    recorder.start()

    recorder.run_session()

    assert camera.opened_with == [str(tmp_path / (CLIP + '-3.bag'))]
    assert (tmp_path / (CLIP + '.bag')).read_bytes() == b'earlier clip'
    assert read_details(tmp_path, CLIP + '-3')['clip'] == CLIP + '-3'
