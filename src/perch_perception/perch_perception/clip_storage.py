"""Keep recorded clips on Google Drive.

    python -m perch_perception.clip_storage upload --domain indoor
    python -m perch_perception.clip_storage download --domain indoor NAME [NAME ...]
    python -m perch_perception.clip_storage download --domain indoor --missing
    python -m perch_perception.clip_storage cleanup --domain indoor [--yes]

--domain is indoor or outdoor and picks both the local and the Drive folder.

A clip is every file in the recordings folder that shares a name and ends in
.bag, .mp4 or .json, for example 2026-10-15T14-02-11.bag and
2026-10-15T14-02-11.mp4. upload sends finished clips that are not on the
Drive yet, download fetches clips from it, and cleanup deletes local clips
only once every file of the clip is on the Drive with a matching checksum.

The copying is done by rclone, a separate program that has to be installed
and logged in to Google Drive on each machine (see the package README).
These commands never overwrite or delete anything on the Drive.
"""

import argparse
import glob
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from typing import NamedTuple, Optional

from perch_perception.record import default_folder

CLIP_EXTENSIONS = ('.bag', '.mp4', '.json')
STILL_WRITING_SECONDS = 60  # a clip changed more recently than this is left alone
DRIVE_FOLDER = 'vespr-drive:VESPR Recordings'  # the clip folder in John's Google Drive
DOMAINS = {'indoor': 'Indoors', 'outdoor': 'Outdoors'}  # each domain's subfolder on the Drive
RCLONE_DIRECTORY_NOT_FOUND = 3  # rclone's exit code when a folder does not exist
SETUP_HELP = 'see "Google Drive storage" in the README'


class LocalFile(NamedTuple):
    """A clip file on this machine."""

    name: str  # file name, for example '2026-10-15T14-02-11.bag'
    size: int  # bytes
    modified: float  # last modification, seconds since the epoch


class RemoteFile(NamedTuple):
    """A clip file on the Drive, as rclone lists it."""

    name: str
    size: int  # bytes
    md5: Optional[str]  # checksum the Drive reports, or None if it gave none


class CopyPlan(NamedTuple):
    """What to do with one clip's files when copying the clip somewhere."""

    to_copy: list  # file names missing at the destination
    conflicts: list  # file names the destination already has with a different size


class RcloneError(Exception):
    """rclone ran and reported a failure."""

    def __init__(self, message, exit_code):
        super().__init__(message)
        self.exit_code = exit_code


def is_login_error(error):
    """Return True if an RcloneError means the Google login expired or was revoked."""
    return 'oauth2' in str(error) or 'invalid_grant' in str(error)


def clip_of(file_name):
    """Return the clip a file belongs to, or None if it is not a clip file.

    '2026-10-15T14-02-11.bag' belongs to clip '2026-10-15T14-02-11'. Hidden
    files, such as the '._' copies macOS leaves on USB drives, never count.
    """
    if file_name.startswith('.'):
        return None
    stem, extension = os.path.splitext(file_name)
    if extension not in CLIP_EXTENSIONS:
        return None
    return stem


def group_clips(file_names):
    """Return {clip name: its file names} for the clip files among file_names.

    Both the clips and the files of each clip come out sorted, so clips are
    handled in the order they were recorded.
    """
    clips = {}
    for file_name in sorted(file_names):
        clip = clip_of(file_name)
        if clip is not None:
            clips.setdefault(clip, []).append(file_name)
    return clips


def list_local(folder):
    """Return {file name: LocalFile} for the clip files directly in folder.

    Subfolders are ignored. A folder that does not exist has no clips.
    """
    if not os.path.isdir(folder):
        return {}
    files = {}
    for entry in os.scandir(folder):
        if entry.is_file() and clip_of(entry.name) is not None:
            stat = entry.stat()
            files[entry.name] = LocalFile(entry.name, stat.st_size, stat.st_mtime)
    return files


def still_being_written(files, now):
    """Return True if any of a clip's LocalFiles changed in the last minute.

    The recorder writes a .bag continuously while it records, so a clip that
    is being recorded always looks like this.
    """
    return any(now - file.modified < STILL_WRITING_SECONDS for file in files)


def plan_copy(sizes, destination_sizes):
    """Decide which of one clip's files to copy.

    sizes maps each file of the clip to its size where it is now, and
    destination_sizes maps file names to sizes where it would go. A file
    missing at the destination is copied. One there with the same size is
    taken to be the same file and left alone. One there with a different
    size is a conflict: something else already has that name.
    """
    to_copy = []
    conflicts = []
    for name in sorted(sizes):
        if name not in destination_sizes:
            to_copy.append(name)
        elif destination_sizes[name] != sizes[name]:
            conflicts.append(name)
    return CopyPlan(to_copy, conflicts)


def file_md5(path):
    """Return a file's MD5 checksum as lowercase hex, the form the Drive reports.

    The file is read one megabyte at a time, so a 1 GB clip does not need
    1 GB of memory.
    """
    digest = hashlib.md5()
    with open(path, 'rb') as file:
        while True:
            block = file.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def deletion_problem(file_paths, remote_files):
    """Return why a clip must be kept, or None if it is safe to delete.

    file_paths are the local paths of every file of the clip, and
    remote_files is the Drive listing. The clip is safe only if each file is
    on the Drive with the same MD5 checksum. A matching size is not enough.
    """
    for path in sorted(file_paths):
        name = os.path.basename(path)
        remote = remote_files.get(name)
        if remote is None:
            return f'{name} is not on the Drive'
        if remote.md5 is None:
            return f'the Drive gives no checksum for {name}'
        try:
            local_md5 = file_md5(path)
        except OSError as error:
            return f'{name} could not be read ({error.strerror})'
        if local_md5 != remote.md5:
            return f'{name} differs from the copy on the Drive'
    return None


def remove_partial_downloads(folder, file_name):
    """Delete what an interrupted download of file_name left in folder.

    rclone downloads into '<file name>.<8 hex digits>.partial' and renames it
    when done. If rclone is killed, as when Ctrl+C stops this command, the
    partial file stays behind.
    """
    pattern = os.path.join(glob.escape(folder), glob.escape(file_name) + '.*.partial')
    for path in glob.glob(pattern):
        os.remove(path)


def remote_name(remote):
    """Return the rclone remote a location is on, or None for a local folder.

    'vespr-drive:VESPR Recordings/Indoors' is on the remote 'vespr-drive:'. A
    plain path such as '/tmp/drive' is a local folder, which rclone also
    accepts.
    """
    if os.path.isabs(remote) or ':' not in remote:
        return None
    return remote.split(':', 1)[0] + ':'


def remote_path(remote, file_name):
    """Return the rclone location of one file in the remote folder."""
    if remote.endswith((':', '/')):
        return remote + file_name
    return remote + '/' + file_name


def parse_listing(text):
    """Turn the JSON 'rclone lsjson' prints into {file name: RemoteFile}.

    Only clip files count. An entry looks like
    {"Name": "a.bag", "Size": 742, "IsDir": false, "Hashes": {"md5": "..."}}.
    """
    files = {}
    for entry in json.loads(text):
        name = entry['Name']
        if entry.get('IsDir') or clip_of(name) is None:
            continue
        md5 = (entry.get('Hashes') or {}).get('md5')
        files[name] = RemoteFile(name, entry['Size'], md5)
    return files


class Rclone:
    """Runs rclone. This is the only code in the module that starts it."""

    def __init__(self, executable='rclone'):
        self.executable = executable

    def setup_problem(self, remote):
        """Return what stops rclone from reaching remote, or None if nothing does."""
        if shutil.which(self.executable) is None:
            return f'rclone is not installed; {SETUP_HELP}'
        name = remote_name(remote)
        if name is not None and name not in self._run(['listremotes']).split():
            return f'the rclone remote {name} is not set up on this machine; {SETUP_HELP}'
        return None

    def list_files(self, remote):
        """Return {file name: RemoteFile} for the clip files in remote.

        A folder that does not exist yet, as before the first upload, is
        empty rather than an error.
        """
        try:
            output = self._run(
                ['lsjson', '--files-only', '--hash', '--hash-type', 'md5', remote])
        except RcloneError as error:
            if error.exit_code == RCLONE_DIRECTORY_NOT_FOUND:
                return {}
            raise
        return parse_listing(output)

    def copy_file(self, source, destination):
        """Copy one file. rclone checks the copy's checksum against the original.

        --ignore-existing makes rclone refuse to replace a file that is
        already at the destination, so nothing is ever overwritten.
        """
        self._run(['copyto', '--ignore-existing', source, destination])

    def _run(self, arguments):
        """Run rclone with arguments and return what it printed.

        Raises RcloneError with the last line rclone printed to stderr,
        which is where it puts the reason for a failure.
        """
        result = subprocess.run(
            [self.executable] + arguments, capture_output=True, text=True)
        if result.returncode != 0:
            lines = result.stderr.strip().splitlines()
            message = lines[-1] if lines else f'rclone exited with code {result.returncode}'
            raise RcloneError(message, result.returncode)
        return result.stdout


def _megabytes(size):
    """Format a size in bytes as megabytes, for example '742 MB'."""
    return f'{size / 1e6:.0f} MB'


def _count(number, word):
    """Return '1 file' or '2 files'."""
    return f'{number} {word}' + ('' if number == 1 else 's')


def _summary(counts):
    """Join the counts that are not zero, for example '3 uploaded, 1 skipped'."""
    parts = [f'{count} {label}' for label, count in counts.items() if count]
    return ', '.join(parts) if parts else 'nothing to do'


def upload(folder, remote, rclone, now=None):
    """Send every finished local clip that is not fully on the Drive yet.

    Prints one line per clip it acts on and a summary. Returns the exit
    code: 0 when everything that should have moved moved, 1 when any clip
    failed or conflicted. now is the current time; tests pass a fixed one.
    """
    now = time.time() if now is None else now
    local = list_local(folder)
    if not local:
        print(f'no clips in {folder}')
        return 0

    remote_sizes = {name: file.size for name, file in rclone.list_files(remote).items()}
    counts = {'uploaded': 0, 'already on the Drive': 0, 'skipped': 0,
              'with conflicts': 0, 'failed': 0}
    for clip, names in group_clips(local).items():
        files = [local[name] for name in names]
        if still_being_written(files, now):
            print(f'skipped  {clip}: still being written')
            counts['skipped'] += 1
            continue

        plan = plan_copy({file.name: file.size for file in files}, remote_sizes)
        if plan.conflicts:
            # Something else on the Drive has this clip's name. Upload none
            # of the clip, so the two recordings never get mixed.
            for name in plan.conflicts:
                print(f'conflict {name}: the Drive has a different file with this name, '
                      'left alone')
            counts['with conflicts'] += 1
            continue
        if not plan.to_copy:
            counts['already on the Drive'] += 1
            continue

        size = sum(local[name].size for name in plan.to_copy)
        what = f'{_count(len(plan.to_copy), "file")}, {_megabytes(size)}'
        # A big clip takes minutes over Wi-Fi, so say what is moving first.
        print(f'uploading {clip} ({what})...')
        try:
            for name in plan.to_copy:
                rclone.copy_file(os.path.join(folder, name), remote_path(remote, name))
        except RcloneError as error:
            if is_login_error(error):
                raise  # every later clip would fail the same way; main says how to log in
            print(f'failed   {clip}: {error}')
            counts['failed'] += 1
            continue
        print(f'uploaded {clip} ({what})')
        counts['uploaded'] += 1

    print(_summary(counts))
    return 1 if counts['with conflicts'] or counts['failed'] else 0


def download(folder, remote, rclone, names=(), missing=False):
    """Fetch clips from the Drive into folder. Never replaces a local file.

    names are clip names. A file name such as '2026-10-15T14-02-11.bag' is
    taken to mean its clip. With missing=True, every clip file on the Drive
    that is not in folder is fetched, which also completes a clip that was
    only partly downloaded. Returns the exit code: 1 when any clip was not
    found, conflicted or failed.
    """
    remote_files = rclone.list_files(remote)
    remote_clips = group_clips(remote_files)
    if missing:
        wanted = list(remote_clips)
    else:
        wanted = [clip_of(os.path.basename(name)) or name for name in names]

    os.makedirs(folder, exist_ok=True)
    local_sizes = {name: file.size for name, file in list_local(folder).items()}
    counts = {'downloaded': 0, 'already here': 0, 'not found': 0,
              'with conflicts': 0, 'failed': 0}
    for clip in dict.fromkeys(wanted):  # each clip once, in the order given
        if clip not in remote_clips:
            print(f'not found {clip}: no such clip on the Drive')
            counts['not found'] += 1
            continue

        sizes = {name: remote_files[name].size for name in remote_clips[clip]}
        plan = plan_copy(sizes, local_sizes)
        if plan.conflicts:
            for name in plan.conflicts:
                print(f'conflict {name}: a different local file has this name, left alone')
            counts['with conflicts'] += 1
            continue
        if not plan.to_copy:
            counts['already here'] += 1
            continue

        size = sum(sizes[name] for name in plan.to_copy)
        what = f'{_count(len(plan.to_copy), "file")}, {_megabytes(size)}'
        print(f'downloading {clip} ({what})...')
        try:
            for name in plan.to_copy:
                remove_partial_downloads(folder, name)
                rclone.copy_file(remote_path(remote, name), os.path.join(folder, name))
        except RcloneError as error:
            if is_login_error(error):
                raise  # every later clip would fail the same way; main says how to log in
            print(f'failed   {clip}: {error}')
            counts['failed'] += 1
            continue
        print(f'downloaded {clip} ({what})')
        counts['downloaded'] += 1

    print(_summary(counts))
    return 1 if counts['not found'] or counts['with conflicts'] or counts['failed'] else 0


def cleanup(folder, remote, rclone, delete=False, now=None):
    """List the local clips that are safely on the Drive, and delete them if delete=True.

    A clip is safe when every one of its files is on the Drive with the same
    MD5 checksum. A clip still being written is always kept. Clips that are
    kept are reported, not treated as errors. Returns the exit code: 1 only
    when a deletion failed.
    """
    now = time.time() if now is None else now
    local = list_local(folder)
    if not local:
        print(f'no clips in {folder}')
        return 0

    remote_files = rclone.list_files(remote)
    safe = failed = freed = 0
    for clip, names in group_clips(local).items():
        files = [local[name] for name in names]
        paths = [os.path.join(folder, name) for name in names]
        size = sum(file.size for file in files)
        if still_being_written(files, now):
            print(f'kept    {clip}: still being written')
            continue
        problem = deletion_problem(paths, remote_files)
        if problem is not None:
            print(f'kept    {clip}: {problem}')
            continue

        if delete:
            try:
                for path in paths:
                    os.remove(path)
            except OSError as error:
                print(f'failed  {clip}: {error}')
                failed += 1
                continue
            print(f'deleted {clip} ({_megabytes(size)})')
        else:
            print(f'safe    {clip} ({_megabytes(size)})')
        safe += 1
        freed += size

    if delete:
        print(f'deleted {_count(safe, "clip")}, freed {_megabytes(freed)}')
    elif safe:
        print(f'{_count(safe, "clip")} safe to delete, {_megabytes(freed)}; '
              'run again with --yes to delete')
    else:
        print('no clips are safe to delete yet')
    if failed:
        print(f'{_count(failed, "clip")} could not be deleted')
    return 1 if failed else 0


def parse_args(argv=None):
    # --domain, --folder and --remote belong to every command, so they can
    # be given after it: 'upload --domain indoor'. A parent parser is
    # argparse's way to share options between subcommands.
    shared = argparse.ArgumentParser(add_help=False)
    shared.add_argument(
        '--domain', required=True, choices=sorted(DOMAINS),
        help='indoor rig or outdoor handheld clips; picks both folders')
    shared.add_argument(
        '--folder',
        help='local clip folder (default: vespr_recordings/<domain> in your home folder)')
    shared.add_argument(
        '--remote',
        help='where the clips are on the Drive, as rclone names it '
             f'(default: {DRIVE_FOLDER}/Indoors or /Outdoors)')

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser(
        'upload', parents=[shared], help='send finished clips that are not on the Drive yet')
    download_parser = commands.add_parser(
        'download', parents=[shared], help='fetch clips from the Drive')
    download_parser.add_argument('names', nargs='*', metavar='NAME', help='clips to fetch')
    download_parser.add_argument(
        '--missing', action='store_true', help='fetch every clip file that is not here yet')
    cleanup_parser = commands.add_parser(
        'cleanup', parents=[shared], help='delete local clips that are safely on the Drive')
    cleanup_parser.add_argument(
        '--yes', action='store_true', help='delete them; without it, only list them')

    args = parser.parse_args(argv)
    if args.command == 'download' and not args.names and not args.missing:
        download_parser.error('name the clips to download, or pass --missing')
    # The domain fills in whichever folder was not given, so indoor and
    # outdoor clips never share a folder.
    if args.folder is None:
        args.folder = os.path.join(default_folder(), args.domain)
    if args.remote is None:
        args.remote = DRIVE_FOLDER + '/' + DOMAINS[args.domain]
    return args


def main(argv=None, rclone=None):
    """Run one command and return its exit code. Tests pass a stand-in rclone."""
    args = parse_args(argv)
    # Run over ssh, stdout is a pipe, which Python sends in large blocks, so
    # nothing would appear until the command ends. Line buffering sends each
    # line as soon as it is printed.
    sys.stdout.reconfigure(line_buffering=True)

    # With the clip folder as its own "Drive", every file matches its own
    # checksum and cleanup would delete every clip.
    same_folder = os.path.realpath(args.remote) == os.path.realpath(args.folder)
    if remote_name(args.remote) is None and same_folder:
        print('--remote is the same folder as --folder; refusing to run')
        return 1

    if rclone is None:
        rclone = Rclone()
    try:
        problem = rclone.setup_problem(args.remote)
        if problem is not None:
            print(problem)
            return 1
        if args.command == 'upload':
            return upload(args.folder, args.remote, rclone)
        if args.command == 'download':
            return download(args.folder, args.remote, rclone, args.names, args.missing)
        return cleanup(args.folder, args.remote, rclone, delete=args.yes)
    except RcloneError as error:
        print(f'rclone failed: {error}')
        if is_login_error(error):
            name = remote_name(args.remote) or args.remote
            print(f'the login may have expired; run: rclone config reconnect {name}')
        return 1
    except KeyboardInterrupt:
        # Ctrl+C. Nothing is half-written on the Drive: it only keeps a file
        # once the whole upload arrives. 130 is the shell's code for Ctrl+C.
        print('\nstopped; run the same command again to carry on')
        return 130


if __name__ == '__main__':
    sys.exit(main())
