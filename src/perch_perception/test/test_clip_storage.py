"""Tests for clip_storage.py, which keeps recorded clips on Google Drive.

No network and no Google account are used. FakeRclone (fake_rclone.py) stands
in for rclone. test_clip_storage_rclone.py runs the real rclone between local
folders.
"""

import hashlib
import os
import sys
import time

import pytest

from fake_rclone import FakeRclone
from perch_perception import clip_storage

NOW = time.time()
OLD = NOW - 3600  # an hour ago: a clip that finished long ago
CLIP = '2026-10-15T14-02-11'
REMOTE = 'vespr-drive:VESPR Recordings/Indoors'


def write_file(folder, name, data=b'data', modified=OLD):
    """Write a file and set its modification time."""
    path = folder / name
    path.write_bytes(data)
    os.utime(path, (modified, modified))
    return path


def write_clip(folder, name=CLIP, modified=OLD):
    """Write a .bag and an .mp4 for one clip."""
    write_file(folder, name + '.bag', b'bag data', modified)
    write_file(folder, name + '.mp4', b'mp4', modified)


def test_a_file_belongs_to_the_clip_named_before_its_extension():
    assert clip_storage.clip_of(CLIP + '.bag') == CLIP
    assert clip_storage.clip_of(CLIP + '.mp4') == CLIP
    assert clip_storage.clip_of(CLIP + '.json') == CLIP


def test_files_with_other_extensions_belong_to_no_clip():
    assert clip_storage.clip_of('notes.txt') is None
    assert clip_storage.clip_of(CLIP + '.bag.partial') is None


def test_hidden_files_belong_to_no_clip():
    # macOS leaves '._' copies next to files on USB drives.
    assert clip_storage.clip_of('._' + CLIP + '.bag') is None
    assert clip_storage.clip_of('.DS_Store') is None


def test_files_are_grouped_into_clips_by_name():
    names = ['b.mp4', 'a.bag', 'notes.txt', 'b.bag', 'a.mp4']

    assert clip_storage.group_clips(names) == {
        'a': ['a.bag', 'a.mp4'],
        'b': ['b.bag', 'b.mp4'],
    }


def test_the_local_listing_has_only_clip_files_directly_in_the_folder(tmp_path):
    write_file(tmp_path, CLIP + '.bag', b'12345')
    write_file(tmp_path, 'notes.txt')
    (tmp_path / 'old').mkdir()
    write_file(tmp_path / 'old', 'other.bag')

    files = clip_storage.list_local(str(tmp_path))

    assert list(files) == [CLIP + '.bag']
    assert files[CLIP + '.bag'].size == 5
    assert files[CLIP + '.bag'].modified == pytest.approx(OLD)


def test_a_folder_that_does_not_exist_has_no_clips(tmp_path):
    assert clip_storage.list_local(str(tmp_path / 'missing')) == {}


def test_a_clip_changed_in_the_last_minute_is_still_being_written():
    files = [
        clip_storage.LocalFile(CLIP + '.bag', 10, NOW - 5),
        clip_storage.LocalFile(CLIP + '.mp4', 10, OLD),
    ]

    assert clip_storage.still_being_written(files, NOW)


def test_a_clip_unchanged_for_over_a_minute_is_finished():
    files = [clip_storage.LocalFile(CLIP + '.bag', 10, NOW - 61)]

    assert not clip_storage.still_being_written(files, NOW)


def test_files_missing_at_the_destination_are_copied():
    plan = clip_storage.plan_copy({'a.bag': 10, 'a.mp4': 5}, {})

    assert plan.to_copy == ['a.bag', 'a.mp4']
    assert plan.conflicts == []


def test_a_file_with_the_same_size_at_the_destination_is_not_copied_again():
    plan = clip_storage.plan_copy({'a.bag': 10, 'a.mp4': 5}, {'a.bag': 10})

    assert plan.to_copy == ['a.mp4']


def test_a_file_with_a_different_size_at_the_destination_is_a_conflict():
    plan = clip_storage.plan_copy({'a.bag': 10}, {'a.bag': 99})

    assert plan.to_copy == []
    assert plan.conflicts == ['a.bag']


def test_file_md5_is_the_hex_checksum_the_drive_uses(tmp_path):
    path = write_file(tmp_path, 'a.bag', b'hello')

    assert clip_storage.file_md5(str(path)) == '5d41402abc4b2a76b9719d911017c592'


def remote_copy(path):
    """The RemoteFile the Drive would list for an exact copy of path."""
    data = path.read_bytes()
    return clip_storage.RemoteFile(path.name, len(data), hashlib.md5(data).hexdigest())


def test_a_clip_is_safe_to_delete_when_every_file_matches_the_drive(tmp_path):
    paths = [write_file(tmp_path, 'a.bag', b'bag'), write_file(tmp_path, 'a.mp4', b'mp4')]
    remote = {path.name: remote_copy(path) for path in paths}

    assert clip_storage.deletion_problem([str(path) for path in paths], remote) is None


def test_a_clip_with_a_file_missing_on_the_drive_is_kept(tmp_path):
    bag = write_file(tmp_path, 'a.bag', b'bag')
    mp4 = write_file(tmp_path, 'a.mp4', b'mp4')
    remote = {'a.bag': remote_copy(bag)}

    problem = clip_storage.deletion_problem([str(bag), str(mp4)], remote)

    assert problem == 'a.mp4 is not on the Drive'


def test_a_clip_whose_drive_copy_differs_is_kept(tmp_path):
    bag = write_file(tmp_path, 'a.bag', b'bag')
    different = hashlib.md5(b'BAG').hexdigest()  # same size, other content
    remote = {'a.bag': clip_storage.RemoteFile('a.bag', 3, different)}

    problem = clip_storage.deletion_problem([str(bag)], remote)

    assert problem == 'a.bag differs from the copy on the Drive'


def test_a_clip_is_kept_when_the_drive_gives_no_checksum(tmp_path):
    bag = write_file(tmp_path, 'a.bag', b'bag')
    remote = {'a.bag': clip_storage.RemoteFile('a.bag', 3, None)}

    problem = clip_storage.deletion_problem([str(bag)], remote)

    assert problem == 'the Drive gives no checksum for a.bag'


# What 'rclone lsjson --files-only --hash --hash-type md5' prints, trimmed.
LISTING = '''[
{"Path":"2026-10-15T14-02-11.bag","Name":"2026-10-15T14-02-11.bag","Size":742,
 "MimeType":"application/octet-stream","ModTime":"2026-10-15T14:02:31Z","IsDir":false,
 "Hashes":{"md5":"0cc175b9c0f1b6a831c399e269772661"}},
{"Path":"old","Name":"old","Size":-1,"IsDir":true},
{"Path":"notes.txt","Name":"notes.txt","Size":3,"IsDir":false,"Hashes":{"md5":"x"}},
{"Path":"2026-10-15T14-02-11.mp4","Name":"2026-10-15T14-02-11.mp4","Size":12,"IsDir":false}
]'''


def test_the_listing_gives_each_clip_file_its_size_and_md5():
    files = clip_storage.parse_listing(LISTING)

    assert files[CLIP + '.bag'] == clip_storage.RemoteFile(
        CLIP + '.bag', 742, '0cc175b9c0f1b6a831c399e269772661')


def test_the_listing_leaves_out_folders_and_other_files():
    assert sorted(clip_storage.parse_listing(LISTING)) == [CLIP + '.bag', CLIP + '.mp4']


def test_a_listed_file_without_hashes_has_no_md5():
    assert clip_storage.parse_listing(LISTING)[CLIP + '.mp4'].md5 is None


def test_the_remote_name_is_the_part_up_to_the_colon():
    assert clip_storage.remote_name('vespr-drive:VESPR Recordings/Indoors') == 'vespr-drive:'


def test_a_local_folder_is_on_no_remote():
    assert clip_storage.remote_name('/tmp/drive') is None


def test_a_file_location_joins_the_remote_folder_and_the_file_name():
    assert clip_storage.remote_path('vespr-drive:VESPR Recordings/Indoors', 'a.bag') == (
        'vespr-drive:VESPR Recordings/Indoors/a.bag')
    assert clip_storage.remote_path('vespr-drive:', 'a.bag') == 'vespr-drive:a.bag'


def test_a_missing_rclone_is_reported_with_where_to_find_setup():
    rclone = clip_storage.Rclone(executable='rclone-that-is-not-installed')

    problem = rclone.setup_problem('vespr-drive:VESPR Recordings/Indoors')

    assert problem == 'rclone is not installed; see "Google Drive storage" in the README'


def test_a_drive_folder_that_does_not_exist_yet_lists_as_empty(monkeypatch):
    # Before the first upload, rclone reports the folder as not found (exit code 3).
    def not_found(arguments):
        raise clip_storage.RcloneError('directory not found', 3)

    rclone = clip_storage.Rclone()
    monkeypatch.setattr(rclone, '_run', not_found)

    assert rclone.list_files('vespr-drive:VESPR Recordings/Indoors') == {}


def test_other_listing_failures_are_raised(monkeypatch):
    def fails(arguments):
        raise clip_storage.RcloneError('could not connect', 1)

    rclone = clip_storage.Rclone()
    monkeypatch.setattr(rclone, '_run', fails)

    with pytest.raises(clip_storage.RcloneError, match='could not connect'):
        rclone.list_files('vespr-drive:VESPR Recordings/Indoors')


def test_a_copy_never_replaces_a_file_at_the_destination(monkeypatch):
    calls = []
    rclone = clip_storage.Rclone()
    monkeypatch.setattr(rclone, '_run', calls.append)

    rclone.copy_file('/clips/a.bag', 'vespr-drive:VESPR Recordings/Indoors/a.bag')

    assert calls == [
        ['copyto', '--ignore-existing', '/clips/a.bag',
         'vespr-drive:VESPR Recordings/Indoors/a.bag']]


def test_upload_sends_every_file_of_a_finished_clip(tmp_path):
    write_clip(tmp_path)
    rclone = FakeRclone(REMOTE)

    exit_code = clip_storage.upload(str(tmp_path), REMOTE, rclone, now=NOW)

    assert exit_code == 0
    assert rclone.files == {CLIP + '.bag': b'bag data', CLIP + '.mp4': b'mp4'}


def test_upload_sends_only_the_files_not_on_the_drive_yet(tmp_path):
    write_clip(tmp_path)
    rclone = FakeRclone(REMOTE, files={CLIP + '.bag': b'bag data'})

    clip_storage.upload(str(tmp_path), REMOTE, rclone, now=NOW)

    assert rclone.copies == [(str(tmp_path / (CLIP + '.mp4')), REMOTE + '/' + CLIP + '.mp4')]


def test_upload_skips_a_clip_still_being_written(tmp_path, capsys):
    write_clip(tmp_path, modified=NOW - 5)
    rclone = FakeRclone(REMOTE)

    exit_code = clip_storage.upload(str(tmp_path), REMOTE, rclone, now=NOW)

    assert exit_code == 0
    assert rclone.copies == []
    assert f'skipped  {CLIP}: still being written' in capsys.readouterr().out


def test_upload_leaves_a_conflicting_clip_alone(tmp_path, capsys):
    write_clip(tmp_path)
    rclone = FakeRclone(REMOTE, files={CLIP + '.bag': b'another recording'})

    exit_code = clip_storage.upload(str(tmp_path), REMOTE, rclone, now=NOW)

    assert exit_code == 1
    assert rclone.copies == []
    assert rclone.files == {CLIP + '.bag': b'another recording'}
    assert f'conflict {CLIP}.bag' in capsys.readouterr().out


def test_one_failing_clip_does_not_stop_the_others(tmp_path):
    write_clip(tmp_path, 'a')
    write_clip(tmp_path, 'b')
    rclone = FakeRclone(REMOTE, fail_on={'a.bag'})

    exit_code = clip_storage.upload(str(tmp_path), REMOTE, rclone, now=NOW)

    assert exit_code == 1
    assert sorted(rclone.files) == ['b.bag', 'b.mp4']


def test_running_upload_again_sends_only_what_is_still_missing(tmp_path):
    write_clip(tmp_path, 'a')
    write_clip(tmp_path, 'b')
    rclone = FakeRclone(REMOTE, fail_on={'a.mp4'})
    clip_storage.upload(str(tmp_path), REMOTE, rclone, now=NOW)
    rclone.fail_on.clear()
    rclone.copies.clear()

    exit_code = clip_storage.upload(str(tmp_path), REMOTE, rclone, now=NOW)

    assert exit_code == 0
    assert rclone.copies == [(str(tmp_path / 'a.mp4'), REMOTE + '/a.mp4')]


def test_upload_prints_a_line_per_clip_and_a_summary(tmp_path, capsys):
    write_clip(tmp_path, 'a')
    write_clip(tmp_path, 'b', modified=NOW - 5)

    clip_storage.upload(str(tmp_path), REMOTE, FakeRclone(REMOTE), now=NOW)

    assert capsys.readouterr().out.splitlines() == [
        'uploading a (2 files, 0 MB)...',
        'uploaded a (2 files, 0 MB)',
        'skipped  b: still being written',
        '1 uploaded, 1 skipped',
    ]


def test_upload_from_a_folder_that_does_not_exist_says_there_are_no_clips(tmp_path, capsys):
    folder = str(tmp_path / 'missing')

    exit_code = clip_storage.upload(folder, REMOTE, FakeRclone(REMOTE), now=NOW)

    assert exit_code == 0
    assert capsys.readouterr().out == f'no clips in {folder}\n'


def test_download_fetches_the_named_clip(tmp_path):
    rclone = FakeRclone(REMOTE, files={'a.bag': b'bag', 'a.mp4': b'mp4', 'b.bag': b'other'})

    exit_code = clip_storage.download(str(tmp_path), REMOTE, rclone, names=['a'])

    assert exit_code == 0
    assert sorted(os.listdir(tmp_path)) == ['a.bag', 'a.mp4']
    assert (tmp_path / 'a.bag').read_bytes() == b'bag'


def test_download_takes_a_file_name_to_mean_its_clip(tmp_path):
    rclone = FakeRclone(REMOTE, files={'a.bag': b'bag', 'a.mp4': b'mp4'})

    clip_storage.download(str(tmp_path), REMOTE, rclone, names=['a.bag'])

    assert sorted(os.listdir(tmp_path)) == ['a.bag', 'a.mp4']


def test_download_missing_fetches_every_file_not_here_yet(tmp_path):
    write_file(tmp_path, 'a.bag', b'bag')
    rclone = FakeRclone(REMOTE, files={'a.bag': b'bag', 'a.mp4': b'mp4', 'b.bag': b'other'})

    exit_code = clip_storage.download(str(tmp_path), REMOTE, rclone, missing=True)

    assert exit_code == 0
    assert [destination for _, destination in rclone.copies] == [
        str(tmp_path / 'a.mp4'), str(tmp_path / 'b.bag')]


def test_download_never_replaces_a_different_local_file(tmp_path, capsys):
    write_file(tmp_path, 'a.bag', b'a local recording')
    rclone = FakeRclone(REMOTE, files={'a.bag': b'bag'})

    exit_code = clip_storage.download(str(tmp_path), REMOTE, rclone, names=['a'])

    assert exit_code == 1
    assert (tmp_path / 'a.bag').read_bytes() == b'a local recording'
    assert 'conflict a.bag' in capsys.readouterr().out


def test_download_reports_a_clip_that_is_not_on_the_drive(tmp_path, capsys):
    exit_code = clip_storage.download(str(tmp_path), REMOTE, FakeRclone(REMOTE), names=['nope'])

    assert exit_code == 1
    assert 'not found nope' in capsys.readouterr().out


def test_download_creates_the_folder(tmp_path):
    folder = tmp_path / 'new' / 'recordings'
    rclone = FakeRclone(REMOTE, files={'a.bag': b'bag'})

    clip_storage.download(str(folder), REMOTE, rclone, names=['a'])

    assert (folder / 'a.bag').exists()


def put_on_drive(rclone, folder, names):
    """Put exact copies of local files on the fake Drive."""
    for name in names:
        rclone.files[name] = (folder / name).read_bytes()


def test_cleanup_without_yes_deletes_nothing(tmp_path, capsys):
    write_clip(tmp_path)
    rclone = FakeRclone(REMOTE)
    put_on_drive(rclone, tmp_path, [CLIP + '.bag', CLIP + '.mp4'])

    exit_code = clip_storage.cleanup(str(tmp_path), REMOTE, rclone, now=NOW)

    assert exit_code == 0
    assert sorted(os.listdir(tmp_path)) == [CLIP + '.bag', CLIP + '.mp4']
    assert f'safe    {CLIP}' in capsys.readouterr().out


def test_cleanup_with_yes_deletes_only_clips_safely_on_the_drive(tmp_path):
    write_clip(tmp_path, 'a')
    write_clip(tmp_path, 'b')
    rclone = FakeRclone(REMOTE)
    put_on_drive(rclone, tmp_path, ['a.bag', 'a.mp4', 'b.bag'])  # b.mp4 was never uploaded

    exit_code = clip_storage.cleanup(str(tmp_path), REMOTE, rclone, delete=True, now=NOW)

    assert exit_code == 0
    assert sorted(os.listdir(tmp_path)) == ['b.bag', 'b.mp4']


def test_cleanup_keeps_a_clip_still_being_written_even_when_it_is_on_the_drive(tmp_path):
    write_clip(tmp_path, modified=NOW - 5)
    rclone = FakeRclone(REMOTE)
    put_on_drive(rclone, tmp_path, [CLIP + '.bag', CLIP + '.mp4'])

    clip_storage.cleanup(str(tmp_path), REMOTE, rclone, delete=True, now=NOW)

    assert sorted(os.listdir(tmp_path)) == [CLIP + '.bag', CLIP + '.mp4']


def test_cleanup_says_how_much_space_deleting_would_free(tmp_path, capsys):
    write_file(tmp_path, 'a.bag', b'x' * 3_000_000)
    rclone = FakeRclone(REMOTE)
    put_on_drive(rclone, tmp_path, ['a.bag'])

    clip_storage.cleanup(str(tmp_path), REMOTE, rclone, now=NOW)

    assert capsys.readouterr().out.splitlines()[-1] == (
        '1 clip safe to delete, 3 MB; run again with --yes to delete')


def test_cleanup_reports_why_a_clip_is_kept(tmp_path, capsys):
    write_clip(tmp_path)

    exit_code = clip_storage.cleanup(str(tmp_path), REMOTE, FakeRclone(REMOTE), now=NOW)

    assert exit_code == 0
    assert f'kept    {CLIP}: {CLIP}.bag is not on the Drive' in capsys.readouterr().out


def test_options_can_follow_the_command():
    args = clip_storage.parse_args(
        ['upload', '--domain', 'indoor', '--folder', '/clips', '--remote', '/drive'])

    assert (args.folder, args.remote) == ('/clips', '/drive')


def test_the_domain_picks_the_local_folder_and_the_drive_folder():
    indoor = clip_storage.parse_args(['upload', '--domain', 'indoor'])
    outdoor = clip_storage.parse_args(['upload', '--domain', 'outdoor'])

    assert indoor.remote == 'vespr-drive:VESPR Recordings/Indoors'
    assert outdoor.remote == 'vespr-drive:VESPR Recordings/Outdoors'
    assert indoor.folder == os.path.join(clip_storage.default_folder(), 'indoor')
    assert outdoor.folder == os.path.join(clip_storage.default_folder(), 'outdoor')


def test_the_domain_is_required():
    with pytest.raises(SystemExit):
        clip_storage.parse_args(['upload'])


def test_download_needs_clip_names_or_missing():
    with pytest.raises(SystemExit):
        clip_storage.parse_args(['download', '--domain', 'indoor'])


def test_main_stops_at_a_setup_problem(capsys):
    rclone = FakeRclone(REMOTE, problem='rclone is not installed; see the README')

    exit_code = clip_storage.main(['upload', '--domain', 'indoor'], rclone=rclone)

    assert exit_code == 1
    assert capsys.readouterr().out == 'rclone is not installed; see the README\n'


def test_main_runs_the_command_it_is_given(tmp_path):
    write_clip(tmp_path)
    rclone = FakeRclone(REMOTE)

    argv = ['upload', '--domain', 'indoor', '--folder', str(tmp_path)]

    exit_code = clip_storage.main(argv, rclone=rclone)

    assert exit_code == 0
    assert sorted(rclone.files) == [CLIP + '.bag', CLIP + '.mp4']


class ExpiredLoginRclone(FakeRclone):
    """A runner whose Drive login has expired."""

    def list_files(self, remote):
        raise clip_storage.RcloneError('oauth2: cannot fetch token: invalid_grant', 1)


def test_an_expired_login_gets_the_reconnect_command(tmp_path, capsys):
    argv = ['download', '--missing', '--domain', 'indoor', '--folder', str(tmp_path)]

    exit_code = clip_storage.main(argv, rclone=ExpiredLoginRclone(REMOTE))

    output = capsys.readouterr().out
    assert exit_code == 1
    assert 'rclone failed: oauth2: cannot fetch token: invalid_grant' in output
    assert 'rclone config reconnect vespr-drive:' in output


class WatchingRclone(FakeRclone):
    """A runner that notes what had been printed when each copy started."""

    def __init__(self, capsys, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.capsys = capsys
        self.printed_before_copy = []

    def copy_file(self, source, destination):
        self.printed_before_copy.append(self.capsys.readouterr().out)
        super().copy_file(source, destination)


def test_upload_says_which_clip_it_is_sending_before_the_copy_starts(tmp_path, capsys):
    # A 0.75 GB clip takes minutes over Wi-Fi; without this line the terminal stays blank.
    write_clip(tmp_path)
    rclone = WatchingRclone(capsys, REMOTE)

    clip_storage.upload(str(tmp_path), REMOTE, rclone, now=NOW)

    assert rclone.printed_before_copy[0] == f'uploading {CLIP} (2 files, 0 MB)...\n'


def test_download_says_which_clip_it_is_fetching_before_the_copy_starts(tmp_path, capsys):
    rclone = WatchingRclone(capsys, REMOTE, files={'a.bag': b'bag', 'a.mp4': b'mp4'})

    clip_storage.download(str(tmp_path), REMOTE, rclone, names=['a'])

    assert rclone.printed_before_copy[0] == 'downloading a (2 files, 0 MB)...\n'


def test_main_prints_each_line_as_soon_as_it_is_written(tmp_path, capsys):
    # Run over ssh, stdout is a pipe, which Python fills in blocks: nothing
    # would show until the command ends. Line buffering sends each line at once.
    argv = ['upload', '--domain', 'indoor', '--folder', str(tmp_path)]

    clip_storage.main(argv, rclone=FakeRclone(REMOTE))

    assert sys.stdout.line_buffering


def test_cleanup_refuses_when_the_drive_folder_is_the_clip_folder_itself(tmp_path):
    # Every file would match its own checksum, so every clip would look safe to delete.
    write_clip(tmp_path)
    argv = ['cleanup', '--domain', 'indoor', '--folder', str(tmp_path),
            '--remote', str(tmp_path), '--yes']

    exit_code = clip_storage.main(argv)

    assert exit_code == 1
    assert sorted(os.listdir(tmp_path)) == [CLIP + '.bag', CLIP + '.mp4']


@pytest.mark.skipif(os.geteuid() == 0, reason='root can read any file')
def test_cleanup_keeps_a_clip_it_cannot_read_and_carries_on(tmp_path, capsys):
    write_clip(tmp_path, 'a')
    write_clip(tmp_path, 'b')
    rclone = FakeRclone(REMOTE)
    put_on_drive(rclone, tmp_path, ['a.bag', 'a.mp4', 'b.bag', 'b.mp4'])
    os.chmod(tmp_path / 'a.bag', 0)  # no permission to read it

    exit_code = clip_storage.cleanup(str(tmp_path), REMOTE, rclone, delete=True, now=NOW)

    assert exit_code == 0
    assert sorted(os.listdir(tmp_path)) == ['a.bag', 'a.mp4']
    assert 'kept    a: a.bag could not be read' in capsys.readouterr().out


class InterruptedRclone(FakeRclone):
    """A runner whose copy is stopped with Ctrl+C."""

    def copy_file(self, source, destination):
        raise KeyboardInterrupt


def test_ctrl_c_stops_with_a_message_instead_of_a_traceback(tmp_path, capsys):
    write_clip(tmp_path)
    argv = ['upload', '--domain', 'indoor', '--folder', str(tmp_path)]

    exit_code = clip_storage.main(argv, rclone=InterruptedRclone(REMOTE))

    assert exit_code == 130
    assert capsys.readouterr().out.splitlines()[-1] == (
        'stopped; run the same command again to carry on')


def test_download_clears_a_partial_file_left_by_an_interrupted_download(tmp_path):
    # rclone writes a download to <name>.<8 hex digits>.partial and renames it
    # when done. If rclone is killed, the partial file stays behind.
    write_file(tmp_path, 'a.bag.d15abd53.partial')
    write_file(tmp_path, 'b.bag.0badc0de.partial')  # another clip's, left alone
    rclone = FakeRclone(REMOTE, files={'a.bag': b'bag', 'a.mp4': b'mp4'})

    clip_storage.download(str(tmp_path), REMOTE, rclone, names=['a'])

    assert sorted(os.listdir(tmp_path)) == ['a.bag', 'a.mp4', 'b.bag.0badc0de.partial']


class LoginExpiresRclone(FakeRclone):
    """A runner whose Drive login expires after the listing, during the first copy."""

    def copy_file(self, source, destination):
        self.copies.append((source, destination))
        raise clip_storage.RcloneError('oauth2: cannot fetch token: invalid_grant', 1)


def test_a_login_that_expires_during_an_upload_gets_the_reconnect_command(tmp_path, capsys):
    write_clip(tmp_path, 'a')
    write_clip(tmp_path, 'b')
    rclone = LoginExpiresRclone(REMOTE)
    argv = ['upload', '--domain', 'indoor', '--folder', str(tmp_path)]

    exit_code = clip_storage.main(argv, rclone=rclone)

    assert exit_code == 1
    assert 'rclone config reconnect vespr-drive:' in capsys.readouterr().out
    assert len(rclone.copies) == 1  # every later copy would fail too, so it stops
