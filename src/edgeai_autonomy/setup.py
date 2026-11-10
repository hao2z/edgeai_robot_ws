from setuptools import setup
from glob import glob
setup(
    name='edgeai_autonomy', version='0.1.0', packages=['edgeai_autonomy'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/edgeai_autonomy']),
                ('share/edgeai_autonomy', ['package.xml']),
                ('share/edgeai_autonomy/config', glob('config/*')),
                ('share/edgeai_autonomy/launch', glob('launch/*')),
                ('share/edgeai_autonomy/models/aruco_practice', glob('models/aruco_practice/*'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='EdgeAI team', maintainer_email='team@example.com',
    description='Single-robot exploration, ArUco localization and practice rescue', license='Apache-2.0',
    entry_points={'console_scripts': [
        'aruco_detector = edgeai_autonomy.detector:main',
        'drive_odometry = edgeai_autonomy.drive_odometry:main',
        'velocity_guard = edgeai_autonomy.guard:main',
        'mission = edgeai_autonomy.mission:main',
        'demo = edgeai_autonomy.demo:main',
    ]},
)
