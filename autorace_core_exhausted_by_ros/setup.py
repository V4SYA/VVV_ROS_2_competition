import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'autorace_core_exhausted_by_ros'



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
     (os.path.join('share', package_name, 'calibration'), glob('calibration/*'))
   ],
    install_requires=['setuptools',
                      'ultralytics',],
    zip_safe=True,
    maintainer='Vasya',
    maintainer_email='v.postnykh@g.nsu.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_line=autorace_core_exhausted_by_ros.detect_line:main',
            'detect_sign=autorace_core_exhausted_by_ros.detect_signs:main',
            'pid=autorace_core_exhausted_by_ros.pid:main'
        ],
    },
)
