from glob import glob

from setuptools import find_packages, setup

package_name = 'vespr_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocap_sim = vespr_sim.mocap_sim:main',
            'fake_branch_detector = vespr_sim.fake_branch_detector:main',
            'fake_vio = vespr_sim.fake_vio:main',
        ],
    },
)
