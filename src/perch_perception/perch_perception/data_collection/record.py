"""Record clips from the D435i through a browser page.

    python -m perch_perception.data_collection.record --domain indoor
    then open http://localhost:8080

--domain is indoor (the indoor rig) or outdoor (handheld) and picks the clip
folder, ~/vespr_recordings/indoor or ~/vespr_recordings/outdoor. On macOS,
run it with sudo. To control a recorder on the Jetson from a Mac, see
"Recording from the Mac" in the package README.

The page shows the live left IR image, a Record / Stop button, fields for the
site, branch, light and notes, and a button to mark the last clip bad. Every
clip is named by its number and its start time, for example
0042_2026-10-23T14-02-11, and saved as three files:

    <name>.bag   raw depth and left IR. This is the data.
    <name>.mp4   a small grayscale video for watching the clip. It is
                 lossy, so never use it for training or measurement.
    <name>.json  the clip's details: number, domain, start time, length,
                 the page's fields, and whether it is marked bad.

Clip numbers come from one counter file, ~/vespr_recordings/next_clip_id,
shared by both domains. A number is never handed out twice, so it can be
written down while recording and used later to find the clip.

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
import re
import sys
import threading
import time
from datetime import datetime

import cv2

from perch_perception.process_source import ProcessSource

DOMAINS = ('indoor', 'outdoor')  # evaluated separately, so their clips are kept apart
DETAIL_FIELDS = ('site', 'branch', 'light', 'notes')  # typed on the page, kept from clip to clip

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
  #status { font-size: 1.6em; }
  #last { font-size: 1.3em; }
  label { margin: 0 0.5em; }
  input { font-size: 1em; width: 8em; }
  #notes { width: 16em; }
  #error, #refused { color: #f66; }
</style>
</head>
<body>
<img id="view" src="/stream" alt="live camera view">
<div>
  <label>Site <input id="site"></label>
  <label>Branch <input id="branch"></label>
  <label>Light <input id="light" list="lights"></label>
  <label>Notes <input id="notes"></label>
  <datalist id="lights">
    <option>sun</option><option>overcast</option><option>dusk</option>
    <option>backlit</option><option>indoor lights</option>
  </datalist>
</div>
<div><button id="button">Record</button></div>
<div id="status">Connecting...</div>
<div id="last"></div>
<div><button id="bad" hidden>Mark bad</button></div>
<div id="error"></div>
<div id="refused"></div>
<script>
const FIELDS = ['site', 'branch', 'light', 'notes'];
const view = document.getElementById('view');
const button = document.getElementById('button');
const bad = document.getElementById('bad');
let state = 'preview';
let lastBad = false;
let filled = false;

// A clip's number, the part of its name before the first '_', for example #0042.
function number(clip) {
  return '#' + clip.split('_')[0];
}

// Redraw the page from the status the server reports.
function show(status) {
  state = status.state;
  lastBad = status.last_bad;
  const recording = state === 'recording' || state === 'stopping';
  button.textContent = recording ? 'Stop' : 'Record';
  // The camera restarts when recording starts or stops; no clicks meanwhile.
  button.disabled = state === 'starting' || state === 'stopping';
  view.className = state === 'recording' ? 'recording' : '';
  const lines = {
    preview: 'Ready',
    starting: 'Starting the camera...',
    recording: 'Recording ' + (status.clip ? number(status.clip) : '') + '   ' + status.seconds + ' s',
    stopping: 'Saving...',
  };
  document.getElementById('status').textContent = lines[state];
  document.getElementById('last').textContent = status.last_clip
    ? 'Last clip: ' + number(status.last_clip) + (status.last_bad ? ' (marked bad)' : '')
      + '   ' + status.folder + '/' + status.last_clip + '.bag' : '';
  // Only the last clip can be marked, and only between clips.
  bad.hidden = !status.last_clip || state !== 'preview';
  if (status.last_clip) {
    bad.textContent = (status.last_bad ? 'Unmark ' : 'Mark ') + number(status.last_clip) + ' bad';
  }
  document.getElementById('error').textContent = status.error || '';
  // Fill the fields once, from what the recorder kept, so reloading the page keeps them.
  if (!filled) {
    FIELDS.forEach(field => { document.getElementById(field).value = status.details[field]; });
    filled = true;
  }
}

// Send a request, with a JSON body if one is given. A refusal is shown until the next request.
function send(path, body) {
  const refused = document.getElementById('refused');
  refused.textContent = '';
  const options = {method: 'POST'};
  if (body !== undefined) {
    options.headers = {'Content-Type': 'application/json'};
    options.body = JSON.stringify(body);
  }
  fetch(path, options)
    .then(reply => reply.json().then(answer => {
      if (reply.ok) {
        show(answer);
      } else {
        refused.textContent = answer.error;
      }
    }))
    .catch(() => { refused.textContent = 'The recorder did not answer'; });
}

button.onclick = () => {
  if (state === 'recording') {
    send('/stop');
    return;
  }
  const details = {};
  FIELDS.forEach(field => { details[field] = document.getElementById(field).value; });
  send('/start', details);
};
bad.onclick = () => send('/bad', {bad: !lastBad});
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


class CannotMark(ValueError):
    """The last clip cannot be marked now: there is none yet, a clip is recording, or it is gone."""


def clip_name(number, now):
    """Name a clip by its number and start time, for example 0042_2026-10-10T14-32-05.

    The number comes first, padded to 4 digits, so clips sort by number. Colons are left out of
    the time so the name is valid on every file system.
    """
    return f'{number:04d}_{now:%Y-%m-%dT%H-%M-%S}'


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


def domain_folder(domain):
    """Return the clip folder of a domain, for example ~/vespr_recordings/indoor."""
    return os.path.join(default_folder(), domain)


def counter_path():
    """Return the clip counter's file, next to the domain folders, so both domains share it."""
    return os.path.join(default_folder(), 'next_clip_id')


def write_file_atomically(path, text):
    """Replace a file's contents in one step, so a power cut never leaves it half written.

    The text goes to a temporary file first and is flushed to the disk (fsync); os.replace then
    swaps it in for the old file in a single operation. The drone's battery can be pulled at
    any moment, which is why the flush matters.
    """
    make_folders(os.path.dirname(path) or '.')
    temporary = path + '.tmp'
    with open(temporary, 'w') as file:
        file.write(text)
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, path)
    hand_to_user(path)


def take_clip_number(path):
    """Return the next clip number from the counter file at path, and store the one after it.

    The count is kept in a file, not worked out from the clips in the folder, because
    clip_storage cleanup deletes clips once they are on the Drive: a number is never handed out
    twice, even after its clip is deleted. A missing file starts the count at 1. Raises
    RuntimeError naming the file, and changes nothing, if it holds anything but a number from 1.
    """
    try:
        with open(path) as file:
            text = file.read()
    except FileNotFoundError:
        number = 1
    else:
        if not re.fullmatch(r'[0-9]+', text.strip()) or int(text) < 1:
            raise RuntimeError(
                f'{path} should hold the next clip number, but holds {text!r}. '
                'Write the next number into it, higher than every clip so far.')
        number = int(text)
    write_file_atomically(path, f'{number + 1}\n')
    return number


def clean_details(details):
    """Return the page's fields, {field: text}, with spaces trimmed and missing fields ''.

    Raises ValueError naming the field for one that is not in DETAIL_FIELDS or not text.
    """
    if not isinstance(details, dict):
        raise ValueError('the details must be a JSON object of text fields')
    for field, value in details.items():
        if field not in DETAIL_FIELDS:
            raise ValueError(f'{field!r} is not a detail field: {", ".join(DETAIL_FIELDS)}')
        if not isinstance(value, str):
            raise ValueError(f'the detail {field!r} must be text')
    return {field: details.get(field, '').strip() for field in DETAIL_FIELDS}


def write_details(path, details):
    """Write a clip's details file, <name>.json."""
    write_file_atomically(path, json.dumps(details, indent=2) + '\n')


def free_clip_name(folder, name):
    """Return name, or name-2, name-3 and so on if folder already has a clip by that name.

    The clip number makes names unique, but if the counter file is lost, as when
    the Jetson is reinstalled, numbers start again, and without internet the
    clock can repeat a time too. librealsense would then write over the earlier
    .bag.
    """
    taken = {os.path.splitext(entry)[0] for entry in os.listdir(folder)}
    candidate = name
    number = 1
    while candidate in taken:
        number += 1
        candidate = f'{name}-{number}'
    return candidate


def make_folders(path):
    """Create a folder and any missing folders above it, each handed back under sudo."""
    missing = []
    while path and not os.path.isdir(path):
        missing.append(path)
        path = os.path.dirname(path)
    for folder in reversed(missing):  # the top one first
        os.makedirs(folder, exist_ok=True)
        hand_to_user(folder)


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

    counter is the clip counter's file (see take_clip_number), and domain goes
    into every clip's details.

    Two threads use a Recorder: the camera loop (run) and the web server
    (start, stop, mark_last_clip_bad, status, latest_jpeg). The lock guards what
    they share.
    """

    def __init__(self, open_source, folder, counter, clock=datetime.now, domain=None):
        self._open_source = open_source
        self._folder = str(folder)
        self._counter = str(counter)
        self._clock = clock
        self._domain = domain
        self._details = clean_details({})  # the page's fields, for the next clip
        self._last_details = None  # the details of the last clip, as they should be saved
        self._last_bad = False  # whether its .json, as saved, says bad
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

    def start(self, details=None):
        """Ask for a recording. The camera loop acts on it.

        details, the page's fields as {field: text}, replaces the fields kept from the clip
        before; None keeps them. Raises ValueError, asking for nothing, if a field is unknown
        or not text.
        """
        cleaned = None if details is None else clean_details(details)
        with self._lock:
            if cleaned is not None:
                self._details = cleaned
            self._wanted = True
            self._error = None

    def stop(self):
        """Ask for the recording to end."""
        with self._lock:
            self._wanted = False

    def mark_last_clip_bad(self, bad):
        """Mark the last clip bad (True) or not (False) in its .json. Nothing is deleted.

        extract_frames skips clips marked bad. Raises ValueError if bad is not True or False, and
        CannotMark if this recorder has not recorded a clip yet, if a clip is being recorded (the
        page only offers the button between clips), or if the last clip's .bag is gone, as after
        clip_storage cleanup: a .json on its own would be a clip of its own on the next upload.
        """
        if not isinstance(bad, bool):
            raise ValueError('bad must be true or false')
        with self._lock:
            if self._last_details is None:
                raise CannotMark('there is no clip to mark yet')
            if self._clip is not None or self._wanted:
                raise CannotMark('a clip can only be marked between clips')
            clip = self._last_details['clip']
            if not os.path.exists(os.path.join(self._folder, clip + '.bag')):
                raise CannotMark(
                    f'{clip}.bag is no longer in {self._folder}, probably cleaned up after its '
                    'upload. Mark it in its .json on the Mac, after downloading it.')
            path = os.path.join(self._folder, clip + '.json')
            details = dict(self._last_details, bad=bad)
            # A range trim saved in the .json since is kept, not written over from memory.
            try:
                with open(path) as file:
                    keep = json.load(file).get('keep')
            except (OSError, ValueError, AttributeError):
                keep = None
            if keep is not None:
                details['keep'] = keep
            write_details(path, details)
            self._last_details = details
            self._last_bad = bad

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
                'last_bad': self._last_bad,
                'details': dict(self._details),
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
        started = self._clock()
        try:
            make_folders(self._folder)
            # The number is taken before the camera opens. If the camera then fails, the number
            # is simply skipped: a gap in the numbers is harmless, a number used twice is not.
            number = take_clip_number(self._counter)
        except (OSError, RuntimeError) as error:
            self._report(error)
            return
        name = free_clip_name(self._folder, clip_name(number, started))
        bag_path = os.path.join(self._folder, name + '.bag')
        video_path = os.path.join(self._folder, name + '.mp4')
        details_path = os.path.join(self._folder, name + '.json')
        with self._lock:
            details = {'clip': name, 'number': number, 'domain': self._domain,
                       'started': started.isoformat(timespec='seconds'), 'seconds': None,
                       **self._details, 'bad': False}
        # Until the clip ends, its .json says bad, so a clip cut short by a pulled battery or a
        # killed process is never taken for a good one. It is written before the camera starts,
        # so every .bag has one.
        unfinished = dict(details, bad=True, error='the recording did not finish')
        try:
            write_details(details_path, unfinished)
        except OSError as error:
            self._report(error)
            return
        video = None
        clip_started = None  # time.monotonic() when the camera opened to record
        first_frame = last_frame = None  # time.monotonic() at the first and the last frame
        finished = False  # True once the camera has closed after a normal end
        quitting = None  # a Ctrl+C during the clip, raised again once the camera has closed
        try:
            # Leaving this with block stops the camera, and stopping the
            # camera is what makes librealsense finish the .bag.
            with self._open_source(bag_path) as source:
                clip_started = time.monotonic()
                with self._lock:
                    self._clip = name
                    self._clip_started = clip_started
                video = PreviewVideo(video_path, source.fps)
                try:
                    for frame in source:
                        self._publish(frame.image)
                        video.write(frame.image)
                        last_frame = time.monotonic()
                        if first_frame is None:
                            first_frame = last_frame
                        if self._should_leave(recording=True):
                            break
                except KeyboardInterrupt as interrupt:
                    # Ctrl+C quits the recorder. The clip ends as Stop would end it, and the
                    # interrupt is raised again below, once the camera has finished the .bag.
                    quitting = interrupt
            finished = True
        except (OSError, RuntimeError) as error:
            # A clip cut short may not replay to the end, so it is marked bad. The person
            # recording can unmark it if it turns out fine.
            details['bad'] = True
            details['error'] = str(error)
            self._report(error)
        finally:
            if video is not None:
                video.close()
            recorded = clip_started is not None
            bad_on_disk = True  # the unfinished .json, until it is replaced
            if recorded:
                # From the first frame to the last, as the .mp4 plays: not from when the camera
                # opened, nor to the end of its shutdown.
                details['seconds'] = (0.0 if first_frame is None
                                      else round(last_frame - first_frame, 1))
                if not finished and not details['bad']:
                    # Neither a normal end nor a camera error: a bug, or a second Ctrl+C while
                    # the camera was closing, which can leave the .bag unfinished.
                    details.update(bad=True, error='the recording did not finish')
                try:
                    write_details(details_path, details)
                    bad_on_disk = details['bad']
                except OSError as error:
                    self._report(f'could not save the details of {name}, so it stays marked bad: '
                                 f'{error}')
            elif os.path.exists(bag_path):
                # The camera failed after creating the .bag. Its .json stays bad, now with the
                # camera's error if there was one.
                if 'error' in details:
                    try:
                        write_details(details_path, dict(unfinished, error=details['error']))
                    except OSError:
                        pass  # the .json written first still says bad
            else:
                # The camera never started and left no .bag: a .json alone is no clip.
                os.remove(details_path)
            with self._lock:
                if recorded:
                    self._last_clip = name
                    self._last_details = details
                    self._last_bad = bad_on_disk  # the page shows what the .json says
                self._clip = None
                # However the clip ended, go back to preview.
                self._wanted = False
            for path in (self._folder, bag_path, video_path):
                if os.path.exists(path):
                    hand_to_user(path)
        if quitting is not None:
            raise quitting

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
            if self.path == '/stop':
                recorder.stop()  # stopping must always work, so any body is ignored
                self._send_status()
                return
            if self.path not in ('/start', '/bad'):
                self.send_error(404)
                return
            try:
                body = self._read_json()
                if self.path == '/start':
                    recorder.start(body)  # no body keeps the fields of the clip before
                else:
                    if not isinstance(body, dict) or set(body) != {'bad'} or not isinstance(
                            body['bad'], bool):
                        raise ValueError('send {"bad": true} or {"bad": false}')
                    recorder.mark_last_clip_bad(body['bad'])
            except CannotMark as error:
                self._send_error(409, error)  # 409 Conflict: fine, but not now
            except ValueError as error:
                self._send_error(400, error)  # 400 Bad Request; bad JSON is a ValueError too
            except OSError as error:
                self._send_error(500, f'could not save the mark: {error}')
            else:
                self._send_status()

        def _read_json(self):
            """Return the request's JSON body, or None if it has none."""
            length = int(self.headers.get('Content-Length') or 0)
            if length == 0:
                return None
            return json.loads(self.rfile.read(length))

        def _send_status(self):
            self._send('application/json', json.dumps(recorder.status()).encode())

        def _send_error(self, code, error):
            self._send('application/json', json.dumps({'error': str(error)}).encode(), code)

        def _send(self, content_type, body, code=200):
            self.send_response(code)
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
        '--domain', required=True, choices=DOMAINS,
        help='indoor rig or outdoor handheld clips; picks the clip folder')
    parser.add_argument(
        '--out',
        help='folder for the clips (default: vespr_recordings/<domain> in your home folder)')
    parser.add_argument('--port', type=int, default=8080, help='port of the page')
    parser.add_argument('--width', type=int, default=848)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument(
        '--fps', type=int, default=None,
        help='default 30 on USB 3 and 10 on USB 2')
    args = parser.parse_args(argv)
    if args.out is None:
        args.out = domain_folder(args.domain)
    return args


def main(argv=None):
    args = parse_args(argv)
    if sys.platform == 'darwin' and os.geteuid() != 0:
        print('note: macOS only opens the camera for root, so run this with sudo')

    def open_session(record_to):
        return ProcessSource(open_camera, (args.width, args.height, args.fps, record_to))

    recorder = Recorder(open_session, args.out, counter_path(), domain=args.domain)
    server = make_server(recorder, args.port)
    # The web server runs on its own thread; the camera loop keeps this one.
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'open http://localhost:{args.port} in your browser')
    print(f'clips are saved to {args.out}, numbered by {counter_path()}')
    # Clip names and start times come from this clock. A Jetson without internet
    # may have the wrong time, so show it before anything is recorded.
    print(f'this computer\'s clock reads {datetime.now():%Y-%m-%d %H:%M}; '
          'clips are named by it, so check it is right')
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
