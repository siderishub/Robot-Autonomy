from setuptools import find_packages, setup

package_name = 'week4'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_localization = week4.lidar_localization:main',
            'move_robot = week4.move_robot:main',
            'tf_publisher = week4.tf_publisher:main',
            'compare_odom = week4.compare_odom:main'
        ],
    },
)
