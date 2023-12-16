from setuptools import find_packages, setup

package_name = 'VVV'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vasya',
    maintainer_email='v.postnykh@g.nsu.ru',
    description='Contains launch files for the robot project',
    license='Contains launch files for our package',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
