"""End-to-end tests of clip_storage.py with the real rclone and no Drive.

rclone accepts a plain local folder wherever it accepts a Drive location, so
these tests copy clips between two temporary folders. That exercises the real
rclone commands, flags and listing format. They are skipped where rclone is
not installed.
"""

import hashlib
import os
import shutil
import time

import pytest

from perch_perception import clip_storage

pytestmark = pytest.mark.skipif(shutil.which('rclone') is None, reason='rclone is not installed')

OLD = time.time() - 3600  # an hour ago: a clip that finished long ago
CLIP = '2026-10-15T14-02-11'


@pytest.fixture(autouse=True)
def empty_rclone_config(tmp_path, monkeypatch):
    """Point rclone at an empty config file, so no real login is used or changed."""
    config = tmp_path / 'rclone.conf'
    config.write_text('')
    monkeypatch.setenv('RCLONE_CONFIG', str(config))


def write_clip(folder):
    folder.mkdir(parents=True, exist_ok=True)
    for extension, data in (('.bag', b'bag data' * 1000), ('.mp4', b'mp4')):
        path = folder / (CLIP + extension)
        path.write_bytes(data)
        os.utime(path, (OLD, OLD))


def test_a_clip_goes_to_the_drive_and_comes_back(tmp_path):
    jetson, mac = tmp_path / 'jetson', tmp_path / 'mac'
    write_clip(jetson)
    drive = str(tmp_path / 'drive')  # does not exist yet, as before the first upload

    where = ['--domain', 'indoor', '--remote', drive]

    uploaded = clip_storage.main(['upload', '--folder', str(jetson)] + where)
    downloaded = clip_storage.main(['download', '--missing', '--folder', str(mac)] + where)

    assert (uploaded, downloaded) == (0, 0)
    for extension in ('.bag', '.mp4'):
        name = CLIP + extension
        assert (mac / name).read_bytes() == (jetson / name).read_bytes()


def test_cleanup_deletes_a_clip_only_once_it_is_on_the_drive(tmp_path):
    jetson = tmp_path / 'jetson'
    write_clip(jetson)
    where = ['--domain', 'indoor', '--folder', str(jetson), '--remote', str(tmp_path / 'drive')]

    clip_storage.main(['cleanup', '--yes'] + where)
    assert sorted(os.listdir(jetson)) == [CLIP + '.bag', CLIP + '.mp4']

    clip_storage.main(['upload'] + where)
    clip_storage.main(['cleanup', '--yes'] + where)
    assert os.listdir(jetson) == []


def test_the_listing_reports_the_same_md5_as_python(tmp_path):
    drive = tmp_path / 'drive'
    write_clip(drive)

    files = clip_storage.Rclone().list_files(str(drive))

    data = (drive / (CLIP + '.bag')).read_bytes()
    assert files[CLIP + '.bag'].md5 == hashlib.md5(data).hexdigest()


def test_a_remote_that_is_not_set_up_is_named():
    problem = clip_storage.Rclone().setup_problem('vespr-test-missing:recordings')

    assert problem.startswith('the rclone remote vespr-test-missing: is not set up')
