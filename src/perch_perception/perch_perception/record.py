"""Record clips from the D435i through a browser page.

    python -m perch_perception.record        # on macOS, run it with sudo
    then open http://localhost:8080

The page shows the live left IR image and a Record / Stop button. Every clip
is saved as two files named by its start time:

    <timestamp>.bag   raw depth and left IR. This is the data.
    <timestamp>.mp4   a small grayscale video for watching the clip. It is
                      lossy, so never use it for training or measurement.

librealsense writes the .bag itself, and only from the moment the camera
starts. So the camera is restarted whenever recording starts or stops. One
run of the camera in one mode, preview or recording, is called a session.

Every session runs in its own process (see process_source.py), because on
macOS librealsense can freeze while shutting the camera down, and that would
freeze this page along with it.
"""

import argparse
import http.server
import json
import os
import sys
import threading
import time
from datetime import datetime

import cv2

from perch_perception.process_source import ProcessSource

PAGE = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>VESPR recorder</title>
<style>
  body { font-family: sans-serif; background: #111; color: #eee; text-align: center; }
  img { max-width: 100%; border: 4px solid #333; }
  img.recording { border-color: #d22; }
  button { font-size: 1.5em; padding: 0.4em 1.5em; margin: 0.6em; }
  #error { color: #f66; }
</style>
</head>
<body>
<img id="view" src="/stream" alt="live camera view">
<div><button id="button">Record</button></div>
<div id="status">Connecting...</div>
<div id="last"></div>
<div id="error"></div>
<script>
const view = document.getElementById('view');
const button = document.getElementById('button');
let state = 'preview';

// Redraw the page from the status the server reports.
function show(status) {
  state = status.state;
  const recording = state === 'recording' || state === 'stopping';
  button.textContent = recording ? 'Stop' : 'Record';
  // The camera restarts when recording starts or stops; no clicks meanwhile.
  button.disabled = state === 'starting' || state === 'stopping';
  view.className = state === 'recording' ? 'recording' : '';
  const lines = {
    preview: 'Ready',
    starting: 'Starting the camera...',
    recording: 'Recording ' + status.clip + '   ' + status.seconds + ' s',
    stopping: 'Saving...',
  };
  document.getElementById('status').textContent = lines[state];
  document.getElementById('last').textContent = status.last_clip
    ? 'Last clip: ' + status.folder + '/' + status.last_clip + '.bag' : '';
  document.getElementById('error').textContent = status.error || '';
}

button.onclick = () => {
  const action = state === 'recording' ? '/stop' : '/start';
  fetch(action, {method: 'POST'}).then(reply => reply.json()).then(show);
};
// Ask the server for its status twice a second. If it does not answer, say
// so instead of leaving the last status on screen.
function recorderGone() {
  document.getElementById('status').textContent = 'The recorder is not responding';
  button.disabled = true;
}
setInterval(
  () => fetch('/status').then(reply => reply.json()).then(show).catch(recorderGone), 500);
</script>
</body>
</html>
"""


def clip_name(now):
    """Name a clip by its start time, for example 2026-10-10T14-32-05."""
    return now.strftime('%Y-%m-%dT%H-%M-%S')


def default_folder():
    """Return the recordings folder in the home folder of whoever runs this.

    It sits directly in the home folder, not on the Desktop or in Documents:
    macOS can sync those two to iCloud Drive, and clips are hundreds of
    megabytes each.

    Under sudo, '~' can mean root's home. sudo keeps the original user's
    name in SUDO_USER, so the home directory is looked up from that.
    """
    home = os.path.expanduser('~' + os.environ.get('SUDO_USER', ''))
    return os.path.join(home, 'vespr_recordings')


def hand_to_user(path):
    """Under sudo, give a file back to the person who ran sudo.

    Files made by a root process belong to root and then need sudo to move
    or delete. sudo keeps the original user's ids in SUDO_UID and SUDO_GID.
    """
    if 'SUDO_UID' in os.environ:
        os.chown(path, int(os.environ['SUDO_UID']), int(os.environ['SUDO_GID']))


class PreviewVideo:
    """Writes grayscale frames to an .mp4 for watching a clip afterwards."""

    def __init__(self, path, fps):
        self._path = path
        self._fps = fps
        self._writer = None  # opened on the first frame, which gives the size

    def write(self, image):
        if self._writer is None:
            height, width = image.shape
            self._writer = self._open(width, height)
        # Video encoders expect three color channels, so repeat the gray one.
        self._writer.write(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))

    def _open(self, width, height):
        # H.264 ('avc1') plays everywhere on a Mac. Some OpenCV builds cannot
        # encode it, so fall back to the older MPEG-4 codec ('mp4v').
        for codec in ('avc1', 'mp4v'):
            writer = cv2.VideoWriter(
                self._path, cv2.VideoWriter_fourcc(*codec), self._fps, (width, height))
            if writer.isOpened():
                return writer
        raise RuntimeError(f'could not open a video writer for {self._path}')

    def close(self):
        if self._writer is not None:
            self._writer.release()


class Recorder:
    """Runs the camera in preview or recording sessions, as the button asks.

    open_source(record_to) opens the camera and returns a FrameSource. Given
    a path it also records a .bag there; given None it only streams.

    Two threads use a Recorder: the camera loop (run) and the web server
    (start, stop, status, latest_jpeg). The lock guards what they share.
    """

    def __init__(self, open_source, folder, clock=datetime.now):
        self._open_source = open_source
        self._folder = str(folder)
        self._clock = clock
        self._lock = threading.Lock()
        self._wanted = False  # what the button last asked for
        self._clip = None  # name of the clip being recorded, or None
        self._clip_started = None  # time.monotonic() when that clip started
        self._last_clip = None
        self._error = None
        self._session_failed = False
        self._jpeg = None  # the latest camera image, JPEG encoded
        self._jpeg_count = 0  # goes up by one with every new image
        self._quit = False

    def start(self):
        """Ask for a recording. The camera loop acts on it."""
        with self._lock:
            self._wanted = True
            self._error = None

    def stop(self):
        """Ask for the recording to end."""
        with self._lock:
            self._wanted = False

    def quit(self):
        """Ask the camera loop to finish."""
        with self._lock:
            self._quit = True

    def status(self):
        """Return what the page needs to draw itself."""
        with self._lock:
            recording = self._clip is not None
            if self._wanted and recording:
                state = 'recording'
            elif self._wanted:
                state = 'starting'  # asked for, camera not restarted yet
            elif recording:
                state = 'stopping'  # stop asked for, clip not closed yet
            else:
                state = 'preview'
            seconds = time.monotonic() - self._clip_started if recording else 0.0
            return {
                'state': state,
                'clip': self._clip,
                'seconds': round(seconds, 1),
                'last_clip': self._last_clip,
                'error': self._error,
                'folder': self._folder,
            }

    def latest_jpeg(self):
        """Return (image counter, JPEG bytes or None) for the live view."""
        with self._lock:
            return self._jpeg_count, self._jpeg

    def run(self):
        """Run sessions one after another until quit() is called."""
        while not self._should_leave_loop():
            self.run_session()
            if self._session_failed:
                time.sleep(1)  # do not hammer a camera that is failing

    def run_session(self):
        """Run the camera in one mode until the button asks for the other."""
        self._session_failed = False
        with self._lock:
            record = self._wanted
        if record:
            self._record_clip()
        else:
            self._preview()

    def _preview(self):
        try:
            with self._open_source(None) as source:
                for frame in source:
                    self._publish(frame.image)
                    if self._should_leave(recording=False):
                        break
        except RuntimeError as error:
            self._report(error)

    def _record_clip(self):
        name = clip_name(self._clock())
        os.makedirs(self._folder, exist_ok=True)
        bag_path = os.path.join(self._folder, name + '.bag')
        video_path = os.path.join(self._folder, name + '.mp4')
        video = None
        try:
            # Leaving this with block stops the camera, and stopping the
            # camera is what makes librealsense finish the .bag.
            with self._open_source(bag_path) as source:
                video = PreviewVideo(video_path, source.fps)
                with self._lock:
                    self._clip = name
                    self._clip_started = time.monotonic()
                for frame in source:
                    self._publish(frame.image)
                    video.write(frame.image)
                    if self._should_leave(recording=True):
                        break
        except RuntimeError as error:
            self._report(error)
        finally:
            if video is not None:
                video.close()
            with self._lock:
                if self._clip is not None:
                    self._last_clip = name
                self._clip = None
                # However the clip ended, go back to preview.
                self._wanted = False
            for path in (self._folder, bag_path, video_path):
                if os.path.exists(path):
                    hand_to_user(path)

    def _publish(self, image):
        encoded, jpeg = cv2.imencode('.jpg', image)
        if encoded:
            with self._lock:
                self._jpeg = jpeg.tobytes()
                self._jpeg_count += 1

    def _should_leave(self, recording):
        """Return True when the session's mode no longer matches the button."""
        with self._lock:
            return self._quit or self._wanted != recording

    def _should_leave_loop(self):
        with self._lock:
            return self._quit

    def _report(self, error):
        self._session_failed = True
        with self._lock:
            # The loop retries every second, so only print a new message.
            if str(error) != self._error:
                print(f'camera error: {error}')
            self._error = str(error)
            self._wanted = False


def open_camera(width, height, fps, record_to):
    """Open the D435i, recording to record_to if it is a path.

    This runs in the child process of a session. pyrealsense2 is imported
    here, not at the top, so the rest of this module can be used and tested
    on a machine without it.
    """
    from perch_perception.realsense_source import LiveSource
    return LiveSource(width, height, fps, record_to=record_to)


def make_server(recorder, port):
    """Build the web server for a recorder. It only listens on this machine."""

    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                self._send('text/html; charset=utf-8', PAGE.encode())
            elif self.path == '/status':
                self._send_status()
            elif self.path == '/stream':
                self._stream()
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path == '/start':
                recorder.start()
            elif self.path == '/stop':
                recorder.stop()
            else:
                self.send_error(404)
                return
            self._send_status()

        def _send_status(self):
            self._send('application/json', json.dumps(recorder.status()).encode())

        def _send(self, content_type, body):
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _stream(self):
            # This is MJPEG: one HTTP response that never ends and carries
            # one JPEG after another. The browser shows each as it arrives.
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            sent = 0
            try:
                while True:
                    count, jpeg = recorder.latest_jpeg()
                    if jpeg is None or count == sent:
                        time.sleep(0.01)  # nothing new yet
                        continue
                    sent = count
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                        + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass  # the browser tab was closed

        def log_message(self, *args):
            pass  # keep the terminal quiet

    server = http.server.ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True  # open streams must not block shutdown
    return server


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        '--out', default=default_folder(),
        help='folder for the clips (default: vespr_recordings in your home folder)')
    parser.add_argument('--port', type=int, default=8080, help='port of the page')
    parser.add_argument('--width', type=int, default=848)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument(
        '--fps', type=int, default=None,
        help='default 30 on USB 3 and 10 on USB 2')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if sys.platform == 'darwin' and os.geteuid() != 0:
        print('note: macOS only opens the camera for root, so run this with sudo')

    def open_session(record_to):
        return ProcessSource(open_camera, (args.width, args.height, args.fps, record_to))

    recorder = Recorder(open_session, args.out)
    server = make_server(recorder, args.port)
    # The web server runs on its own thread; the camera loop keeps this one.
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'open http://localhost:{args.port} in your browser')
    print(f'clips are saved to {args.out}')
    print('press Ctrl+C here to quit')
    try:
        recorder.run()
    except KeyboardInterrupt:
        print('\nstopped')
    finally:
        server.shutdown()
        server.server_close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
