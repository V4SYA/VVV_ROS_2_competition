import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'autorace_core_vvv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vasya',
    maintainer_email='v.postnykh@g.nsu.ru',
    description='Contains launch files for the robot project',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'detect_line=autorace_core_vvv.detect_line:main',
        'pid=autorace_core_vvv.pid:main'
        ],
    },
)
