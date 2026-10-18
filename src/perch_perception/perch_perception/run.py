"""Read a frame source end to end and print what it delivers.

    python -m perch_perception.run               # live camera
    python -m perch_perception.run --bag FILE    # replay a recording

Only the --bag flag differs between the two. Detection and pose are not
implemented yet, so for now this prints a depth summary per frame, which is
enough to check that a camera or a recording produces sane data.
"""

import argparse
import sys
import time

import numpy as np


def depth_summary(depth):
    """Return (median depth in meters, fraction of pixels that have depth).

    A depth of 0 means "unknown", so zeros are dropped before the median.
    The median is None when no pixel has depth.
    """
    valid = depth[depth > 0]
    valid_fraction = valid.size / depth.size
    if valid.size == 0:
        return None, valid_fraction
    return float(np.median(valid)), valid_fraction


def describe(frame):
    """Return a one-line summary of a frame."""
    height, width = frame.image.shape[:2]
    median_m, valid_fraction = depth_summary(frame.depth)
    if median_m is None:
        depth_text = 'no valid depth'
    else:
        depth_text = f'median depth {median_m:.3f} m  valid {valid_fraction:.0%}'
    imu_text = 'imu yes' if frame.imu is not None else 'imu no'
    return f't={frame.timestamp:.3f} s  {width}x{height}  {depth_text}  {imu_text}'


def run(source, max_frames=None, print_every=10):
    """Read frames from source and return how many were read.

    Prints a summary of every print_every-th frame. Stops when the source
    ends, after max_frames frames, or on Ctrl+C.
    """
    count = 0
    try:
        for frame in source:
            if count % print_every == 0:
                print(describe(frame))
            count += 1
            if max_frames is not None and count >= max_frames:
                break
    except KeyboardInterrupt:
        print('stopped by Ctrl+C')
    return count


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        '--bag', metavar='FILE',
        help='replay this .bag recording; without it, use the live camera')
    parser.add_argument('--width', type=int, default=848, help='live camera only')
    parser.add_argument('--height', type=int, default=480, help='live camera only')
    parser.add_argument(
        '--fps', type=int, default=None,
        help='live camera only; default 30 on USB 3 and 10 on USB 2')
    parser.add_argument(
        '--imu', action='store_true',
        help='live camera only: also stream accel and gyro (not on macOS)')
    parser.add_argument(
        '--max-frames', type=int, default=None, help='stop after this many frames')
    parser.add_argument(
        '--print-every', type=int, default=10,
        help='print one summary line every N frames (default 10)')
    return parser.parse_args(argv)


def open_source(args):
    """Build the frame source the command line asked for."""
    # Imported here, not at the top, so the functions above can be used and
    # tested on a machine without pyrealsense2.
    from perch_perception.realsense_source import BagSource, LiveSource

    if args.bag is not None:
        print(f'replaying {args.bag}')
        return BagSource(args.bag)

    source = LiveSource(args.width, args.height, args.fps, args.imu)
    imu_text = 'on' if source.imu_enabled else 'off'
    print(f'live camera, USB {source.usb_type or "unknown"}: '
          f'{args.width}x{args.height} at {source.fps} fps, IMU {imu_text}')
    if not source.usb_type.startswith('3'):
        print('WARNING: not a USB 3 connection, so the frame rate is reduced')
    if args.imu and not source.imu_enabled:
        print('this camera offers no IMU streams; continuing without IMU')
    return source


def main(argv=None):
    args = parse_args(argv)
    try:
        source = open_source(args)
    except ImportError as error:
        print(f'pyrealsense2 is needed to open a camera or a bag: {error}')
        return 1
    except (RuntimeError, FileNotFoundError) as error:
        print(f'could not open the source: {error}')
        return 1

    started = time.monotonic()
    # The with block stops the camera or closes the bag however run() ends.
    with source:
        count = run(source, args.max_frames, args.print_every)
    elapsed = time.monotonic() - started
    print(f'read {count} frames in {elapsed:.1f} s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
