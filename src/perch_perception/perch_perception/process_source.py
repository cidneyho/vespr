"""Run a frame source in a separate process.

On macOS, librealsense can freeze while it shuts the camera down. A freeze
inside the library stops every thread of the process it happens in, so a
program that also serves a web page would stop answering.

ProcessSource gives a source its own short-lived process, the child. The
child opens the source, sends every frame back over a pipe, and when asked
to stop it closes the source and exits at once, skipping the rest of
Python's cleanup. If the child freezes anyway, the parent ends it and
carries on.

No camera SDK is imported here. The child imports whatever the function it
is given needs.
"""

import multiprocessing
import os
import signal
import time

from perch_perception.frame_source import FrameSource


def _child_main(open_source, args, pipe, stop):
    """Everything the child process does."""
    # Ctrl+C in a terminal is sent to the child as well as the parent. The
    # child ignores it and waits for the parent to ask it to stop, so that a
    # recording is always closed properly.
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    try:
        source = open_source(*args)
    except Exception as error:
        pipe.send(('error', str(error)))
        os._exit(1)

    pipe.send(('opened', getattr(source, 'fps', None)))
    try:
        for frame in source:
            pipe.send(('frame', frame))
            if stop.is_set():
                break
    except Exception as error:
        pipe.send(('error', str(error)))

    source.close()
    pipe.send(('closed', None))
    # os._exit ends the process immediately. Python does not run its usual
    # cleanup, which is where librealsense froze on macOS.
    os._exit(0)


class ProcessSource(FrameSource):
    """A frame source that runs open_source(*args) in a child process.

    open_source must be a module-level function, because the child finds it
    by importing its module. The constructor returns once the source has
    opened in the child, or raises RuntimeError with the child's message.
    """

    def __init__(self, open_source, args=(), open_timeout=40, frame_timeout=10,
                 close_timeout=15):
        self._frame_timeout = frame_timeout
        self._close_timeout = close_timeout
        self._closed = False

        # 'spawn' starts the child as a fresh Python interpreter. The other
        # choice, 'fork', copies this process including its threads, which
        # is unsafe for a process that runs a web server.
        context = multiprocessing.get_context('spawn')
        self._pipe, child_pipe = context.Pipe(duplex=False)
        self._stop = context.Event()
        self._process = context.Process(
            target=_child_main, args=(open_source, args, child_pipe, self._stop),
            daemon=True)
        self._process.start()
        self.pid = self._process.pid
        # The child has its own copy of the sending end. Closing ours means
        # the pipe reports end-of-file as soon as the child is gone.
        child_pipe.close()

        try:
            kind, payload = self._receive(open_timeout, 'the source did not open')
            if kind == 'error':
                raise RuntimeError(payload)
        except RuntimeError:
            # However opening failed, end the child. One that is still busy
            # opening would otherwise keep the camera, and every later
            # attempt to open it would fail.
            self._end_process()
            raise
        self.fps = payload

    def frames(self):
        while True:
            kind, payload = self._receive(self._frame_timeout, 'no frames arrived')
            if kind == 'frame':
                yield payload
            elif kind == 'error':
                raise RuntimeError(payload)
            else:  # 'closed': the source ran out of frames
                return

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()

        # Keep reading while the child finishes. It may be in the middle of
        # sending a frame, and it cannot move on until that frame is read.
        deadline = time.monotonic() + self._close_timeout
        while time.monotonic() < deadline and self._process.is_alive():
            if self._pipe.poll(0.05):
                try:
                    kind, _ = self._pipe.recv()
                except EOFError:
                    break
                if kind == 'closed':
                    break

        self._process.join(max(0.0, deadline - time.monotonic()))
        if self._process.is_alive():
            self._end_process()
            raise RuntimeError(
                'the camera process froze while closing and was ended; '
                'a clip recorded in it may be unfinished')

    def _receive(self, timeout, what_is_missing):
        """Return the next (kind, payload) message from the child."""
        if not self._pipe.poll(timeout):
            raise RuntimeError(f'{what_is_missing} within {timeout:g} s')
        try:
            return self._pipe.recv()
        except EOFError:
            raise RuntimeError('the camera process ended unexpectedly') from None

    def _end_process(self):
        """Force the child to end, whatever it is doing."""
        if self._process.is_alive():
            self._process.kill()
        self._process.join()
