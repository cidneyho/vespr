"""Tests for record.py: clip naming, the preview/recording sessions, and the web page.

No camera is used. FakeCamera stands in for the function that opens the camera
and hands out in-memory sources.
"""

import json
import os
import threading
import time
import urllib.request
from datetime import datetime

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from fake_source import ListSource, make_frame  # noqa: E402
from perch_perception.data_collection import record  # noqa: E402

NOW = datetime(2026, 10, 10, 14, 32, 5)
CLIP = '2026-10-10T14-32-05'
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
    """Stands in for the function that opens the camera."""

    def __init__(self, frame_count=5, before_frame=None, callback=None):
        self.frame_count = frame_count
        self.before_frame = before_frame
        self.callback = callback
        self.opened_with = []  # the record_to argument of every open
        self.sources = []

    def __call__(self, record_to):
        self.opened_with.append(record_to)
        frames = [make_frame(float(i), shape=SHAPE) for i in range(self.frame_count)]
        source = ScriptedSource(frames, self.before_frame, self.callback)
        self.sources.append(source)
        return source


def make_recorder(camera, folder):
    return record.Recorder(camera, folder, clock=lambda: NOW)


def count_video_frames(path):
    capture = cv2.VideoCapture(str(path))
    count = 0
    while capture.read()[0]:
        count += 1
    capture.release()
    return count


def test_clip_name_is_an_iso_timestamp_without_colons():
    assert record.clip_name(NOW) == CLIP


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


def post(url):
    request = urllib.request.Request(url, data=b'', method='POST')
    return urllib.request.urlopen(request, timeout=5).status


def test_the_page_has_a_record_button(served):
    _, base = served

    page = urllib.request.urlopen(base + '/', timeout=5).read().decode()

    assert '<button' in page
    assert 'Record' in page


def test_posting_start_requests_a_recording(served):
    recorder, base = served

    assert post(base + '/start') == 200
    assert recorder.status()['state'] == 'starting'


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
