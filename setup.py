from setuptools import setup
import os
from glob import glob

package_name = 'bugbase_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.png'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bugcar_ros2',
    maintainer_email='bugcar_ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odrive_node = bugbase_node.odrive_node:main',
            'connect_gamepad_node = bugbase_node.connect_gamepad:main'
        ],
    },
)
