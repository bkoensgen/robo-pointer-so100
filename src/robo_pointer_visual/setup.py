import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robo_pointer_visual'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools', 'ultralytics'],
    zip_safe=True,
    maintainer='benja',
    maintainer_email='benjamin.koensgen@gmail.com', 
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = robo_pointer_visual.vision_node:main',
            'robot_controller_node = robo_pointer_visual.robot_controller_node:main',
            'real_robot_interface = robo_pointer_visual.real_robot_interface:main'
        ],
    },
)
