from setuptools import find_packages, setup

package_name = 'aruco_create'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/aruco_launch.py']),
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
            'aruco_create = aruco_create.camera_calib:main',
            'aruco_detect_node = aruco_create.aruco_detect_node:main',
            'aruco_odom = aruco_create.odom:main',
            'aruco_control = aruco_create.control:main'
        ],
    },
)
