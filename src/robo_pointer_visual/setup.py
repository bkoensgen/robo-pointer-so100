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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benja',
    maintainer_email='benja@todo.todo', # Pense à mettre ton vrai email plus tard
    description='TODO: Package description', # Pense à mettre une description plus tard
    license='TODO: License declaration', # Pense à choisir une licence plus tard (ex: Apache-2.0)
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = robo_pointer_visual.vision_node:main',
            'robot_controller_node = robo_pointer_visual.robot_controller_node:main',
            'dummy_robot_interface = robo_pointer_visual.dummy_robot_interface:main',
            'real_robot_interface = robo_pointer_visual.real_robot_interface:main',
            'check_env = robo_pointer_visual.check_env:main',
        ],
    },
)
