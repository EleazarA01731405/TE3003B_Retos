from setuptools import find_packages, setup

package_name = 'reto_final_real'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/reto_final_real.py']),
    ('share/' + package_name + '/calibration_matrix', ['calibration_matrix/calibration_matrix.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='puzzlebot',
    maintainer_email='puzzlebot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_create = reto_final_real.camera_calib:main',
            'aruco_detect_node = reto_final_real.aruco_detect_node:main',
            'aruco_odom = reto_final_real.odom:main',
            'aruco_control = reto_final_real.control:main',
            'wall_follower = reto_final_real.wall_follower:main',
        ],
    },
)
