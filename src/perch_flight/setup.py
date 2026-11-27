from glob import glob

from setuptools import find_packages, setup

package_name = 'perch_flight'

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
            'fly_square = perch_flight.fly_square:main',
            'flight_cli = perch_flight.flight_cli:main',
            'external_bridge = perch_flight.external_bridge:main',
        ],
    },
)
