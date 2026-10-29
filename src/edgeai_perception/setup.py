from setuptools import find_packages, setup

package_name = 'edgeai_perception'

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
    maintainer='hector',
    maintainer_email='inbox@haozhang.se',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [         
            'aruco_detector = edgeai_perception.aruco_detector:main',
            'square_patrol = edgeai_perception.square_patrol:main',
        ],
    },
)
