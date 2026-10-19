"""An in-memory stand-in for the rclone runner in clip_storage.py."""

import hashlib

from perch_perception.clip_storage import RcloneError, RemoteFile, clip_of


class FakeRclone:
    """Keeps the Drive's files in memory and records every copy it is asked for.

    remote is the Drive location the commands are given. files maps file
    names on the Drive to their contents. A copy of a file named in fail_on
    raises RcloneError, like a dropped connection. problem is what
    setup_problem reports.
    """

    def __init__(self, remote='vespr-drive:VESPR Recordings/Indoors', files=None, fail_on=(),
                 problem=None):
        self.remote = remote
        self.files = dict(files or {})
        self.fail_on = set(fail_on)
        self.problem = problem
        self.copies = []  # (source, destination) of every copy asked for

    def setup_problem(self, remote):
        return self.problem

    def list_files(self, remote):
        assert remote == self.remote
        return {
            name: RemoteFile(name, len(data), hashlib.md5(data).hexdigest())
            for name, data in self.files.items()
            if clip_of(name) is not None
        }

    def copy_file(self, source, destination):
        self.copies.append((source, destination))
        prefix = self.remote + '/'
        if destination.startswith(prefix):  # an upload
            name = destination[len(prefix):]
            if name in self.fail_on:
                raise RcloneError(f'failed to upload {name}', 1)
            with open(source, 'rb') as file:
                self.files[name] = file.read()
        else:  # a download
            name = source[len(prefix):]
            if name in self.fail_on:
                raise RcloneError(f'failed to download {name}', 1)
            with open(destination, 'wb') as file:
                file.write(self.files[name])
