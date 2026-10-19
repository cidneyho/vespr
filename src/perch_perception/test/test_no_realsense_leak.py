"""Only realsense_source.py may depend on the camera SDK.

If pyrealsense2 leaks into the pose or detection code, that code can no
longer run on recorded data on a machine without the SDK, and it becomes
tied to this one camera.
"""

import subprocess
import sys

import pytest

# Add the detection and pose modules here when they are written.
CAMERA_FREE_MODULES = [
    'perch_perception.clip_storage',
    'perch_perception.frame_source',
    'perch_perception.process_source',
    'perch_perception.record',
    'perch_perception.run',
]


@pytest.mark.parametrize('module', CAMERA_FREE_MODULES)
def test_importing_the_module_does_not_import_pyrealsense2(module):
    # Checked in a fresh interpreter, because other tests in this process
    # have already imported pyrealsense2.
    check = (
        f'import sys, {module}; '
        "sys.exit(1 if 'pyrealsense2' in sys.modules else 0)"
    )

    result = subprocess.run([sys.executable, '-c', check])

    assert result.returncode == 0
