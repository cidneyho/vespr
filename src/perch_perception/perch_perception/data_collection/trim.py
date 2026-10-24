"""Keep only part of a recorded clip, named by its number; extract_frames skips the rest.

    python -m perch_perception.data_collection.trim 42 --keep 0 12.5 --domain outdoor
    python -m perch_perception.data_collection.trim 42 --all --domain outdoor

Nothing is cut out of the .bag, which stays as it was recorded. The range is saved in the clip's
.json as "keep": [start, end], in seconds from the clip's first frame, and the clip is no longer
marked bad, because a trimmed clip is one worth using. Find the times by playing the clip's .mp4.
--all removes the range, so the whole clip is kept again.

Run it on the Mac after downloading the clip and before extract_frames: a clip's .json is only
read the first time the clip is extracted.
"""

import argparse
import json
import math
import re
import sys
from pathlib import Path

from perch_perception.data_collection.record import DOMAINS, domain_folder, write_details


class TrimError(Exception):
    """The clip cannot be found, or the range cannot be kept. The message says why."""


def find_clip(folder, name):
    """Return the .bag of a clip in folder, named by its number (42 or 0042) or its whole name.

    Raises TrimError if there is no such clip, or if two clips share the number, as they can
    after the recorder's counter file is lost.
    """
    folder = Path(folder)
    name = name.removesuffix('.bag')
    if (folder / f'{name}.bag').exists():
        return folder / f'{name}.bag'
    if re.fullmatch(r'[0-9]+', name):
        # A numbered clip's name starts with its number and '_', for example 0042_2026-...
        matches = sorted(path for path in folder.glob('*.bag')
                         if re.match(r'[0-9]+_', path.name)
                         and int(path.name.split('_')[0]) == int(name))
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            raise TrimError(f'{len(matches)} clips have number {int(name)}: '
                            f'{", ".join(path.stem for path in matches)}. Give the whole name.')
    raise TrimError(f'there is no clip {name} in {folder}')


def set_keep(bag_path, start, end):
    """Save the range of a clip to keep in its .json, and return the clip's details.

    start and end are seconds from the clip's first frame; an end past the end of the clip keeps
    the rest of it. The clip is then no longer marked bad. start None removes the range. A clip
    without a .json, recorded before the recorder wrote one, gets one. Raises TrimError,
    changing nothing, if the .json cannot be read, if the range does not run from 0 or later to
    a later end, or if it starts after the clip ends.
    """
    path = Path(bag_path).with_suffix('.json')
    details = {}
    if path.exists():
        try:
            details = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError) as error:  # a JSON syntax error is a ValueError
            raise TrimError(f'{path} cannot be read: {error}') from None
        if not isinstance(details, dict):
            raise TrimError(f'{path} must hold a JSON object, the clip\'s details')

    if start is None:
        details.pop('keep', None)
    else:
        if not (math.isfinite(start) and math.isfinite(end) and 0 <= start < end):
            raise TrimError(f'the range must start at 0 s or later and end after it starts, '
                            f'not {start:g} to {end:g} s')
        # seconds is null for a clip that did not finish, so its length is unknown.
        seconds = details.get('seconds')
        if isinstance(seconds, (int, float)) and not isinstance(seconds, bool) \
                and start >= seconds:
            raise TrimError(f'{path.stem} is {seconds:g} s long, so a range starting at '
                            f'{start:g} s keeps nothing')
        details['keep'] = [start + 0.0, end + 0.0]  # + 0.0 turns -0.0 into 0.0
        details['bad'] = False
    write_details(str(path), details)
    return details


def preview_seconds(bag_path):
    """Return how long the clip's .mp4 plays, in seconds, or None if it cannot be read."""
    import cv2  # imported here, so the rest of this module does not need it

    capture = cv2.VideoCapture(str(Path(bag_path).with_suffix('.mp4')))
    try:
        frames = capture.get(cv2.CAP_PROP_FRAME_COUNT)
        fps = capture.get(cv2.CAP_PROP_FPS)
    finally:
        capture.release()
    return frames / fps if frames > 0 and fps > 0 else None


def warnings_for(bag_path, details):
    """Return what to warn about before trusting a range kept from this clip."""
    warnings = []
    if details.get('error') == 'the recording did not finish':
        warnings.append(
            'this clip did not finish recording, so its .bag may not open at all: librealsense '
            'cannot open a truncated .bag, and trimming does not change that. If extract_frames '
            'fails on it, mark it bad again.')
    seconds = details.get('seconds')
    played = preview_seconds(bag_path)
    if isinstance(seconds, (int, float)) and played is not None and seconds - played > 0.5:
        warnings.append(
            f'the .mp4 plays for {played:g} s, but the clip is {seconds:g} s long: frames were '
            'dropped while recording, so .mp4 times run behind the clip\'s, more so the later in '
            'the clip. Leave a wider margin, especially late in the clip.')
    return warnings


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('clip', help='the clip\'s number, such as 42, or its whole name')
    which = parser.add_mutually_exclusive_group(required=True)
    which.add_argument('--keep', nargs=2, type=float, metavar=('START', 'END'),
                       help='the seconds to keep, as the .mp4 player shows them')
    which.add_argument('--all', action='store_true', help='keep the whole clip again')
    parser.add_argument('--domain', required=True, choices=DOMAINS,
                        help='indoor or outdoor; picks the clip folder')
    parser.add_argument(
        '--folder',
        help='clip folder (default: vespr_recordings/<domain> in your home folder)')
    args = parser.parse_args(argv)
    if args.folder is None:
        args.folder = domain_folder(args.domain)
    return args


def main(argv=None):
    args = parse_args(argv)
    try:
        bag = find_clip(args.folder, args.clip)
        if args.all:
            details = set_keep(bag, None, None)
            still_bad = ' It is marked bad, so it is still skipped.' if details.get('bad') else ''
            print(f'{bag.stem}: the whole clip is kept.{still_bad}')
        else:
            start, end = args.keep
            details = set_keep(bag, start, end)
            seconds = details.get('seconds')
            of = f' of {seconds:g} s' if isinstance(seconds, (int, float)) else ''
            print(f'{bag.stem}: keeping {start:g} to {end:g} s{of}; it is no longer marked bad')
            for warning in warnings_for(bag, details):
                print(f'warning: {warning}')
    except TrimError as error:
        print(f'error: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
