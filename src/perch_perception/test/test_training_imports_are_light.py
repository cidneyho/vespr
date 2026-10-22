"""The training modules must not pull heavy libraries onto the drone.

perch_perception is installed on the Jetson, which has no torch or ultralytics. If a training
module imports one of them at the top of the file, importing it there fails, and the libraries
also slow down every test and command that only needs the dataset files. They are imported
inside the functions that use them instead.
"""

import subprocess
import sys

import pytest

# Each new training module is added here.
TRAINING_MODULES = [
    'perch_perception.training.dataset',
    'perch_perception.training.extract_frames',
    'perch_perception.training.import_labels',
]

HEAVY_LIBRARIES = ['torch', 'ultralytics', 'onnx', 'onnxruntime', 'rclpy']


@pytest.mark.parametrize('module', TRAINING_MODULES)
def test_importing_a_training_module_does_not_load_heavy_libraries(module):
    # Checked in a fresh interpreter, because other tests in this process
    # may already have imported these libraries.
    check = (
        f'import sys, {module}; '
        f'loaded = [name for name in {HEAVY_LIBRARIES!r} if name in sys.modules]; '
        'print(loaded); '
        'sys.exit(1 if loaded else 0)'
    )

    result = subprocess.run([sys.executable, '-c', check], capture_output=True, text=True)

    assert result.returncode == 0, result.stdout + result.stderr
