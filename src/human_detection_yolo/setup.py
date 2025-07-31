from setuptools import setup
import os
from glob import glob

package_name = 'human_detection_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jakgritb',
    maintainer_email='jakgritb@todo.todo',
    description='Human detection using YOLOv8 for TurtleBot3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = human_detection_yolo.yolo_detector:main',
            'human_subscriber = human_detection_yolo.human_subscriber:main',
        ],
    },
)
