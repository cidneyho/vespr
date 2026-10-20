"""imu_log.py may only import pyrealsense2 inside open_camera.

The module and its tests have to import on a machine without the camera
SDK, and the SDK lives in a build folder that is only put on the path when
the camera is opened.
"""

import subprocess
import sys

import pytest

CAMERA_FREE_MODULES = [
    'perch_eval.imu_log',
]


@pytest.mark.parametrize('module', CAMERA_FREE_MODULES)
def test_importing_the_module_does_not_import_pyrealsense2(module):
    # Checked in a fresh interpreter, so nothing else in this test run can
    # have imported pyrealsense2 already.
    check = (
        f'import sys, {module}; '
        "sys.exit(1 if 'pyrealsense2' in sys.modules else 0)"
    )

    result = subprocess.run([sys.executable, '-c', check])

    assert result.returncode == 0
