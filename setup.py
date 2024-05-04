import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'minichallenge4_bueno'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
         (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robertog',
    maintainer_email='robertog@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = minichallenge4_bueno.controller:main',
            'traffic_lights = minichallenge4_bueno.traffic_lights:main',
        ],
    },
)
