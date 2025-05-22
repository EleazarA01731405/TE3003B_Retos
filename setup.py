from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'reto_final_real'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eleazar Olivas | Angel Estrada | Arick Morelos',
    maintainer_email='A01731405@tec.mx | A01732584@tec.mx | A01735192@tec.mx',
    description='Entrega final real',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower = reto_final_real.wall_follower:main',
            'degrees_calibration = reto_final_real.degrees_calibration:main'
        ],
    },
)
