from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'f450_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/world', glob('world/*')),
        (os.path.join('share', package_name, 'models','F450'), glob('models/F450/*', recursive=True)),
        (os.path.join('share', package_name, 'models','Aruco_Target'), glob('models/Aruco_Target/*', recursive=True)),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mars0824',
    maintainer_email='mengyuyuoh@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['pose = f450_simulation.pose:main',
                            'quad_broadcast = f450_simulation.quad_broadcast:main',
                            'takeoff = f450_simulation.takeoff:main',
                            'trajectory = f450_simulation.trajectory:main',
                            'forward = f450_simulation.forward:main',
                            'circlepid = f450_simulation.circlepid:main',
        ],
    },
)
