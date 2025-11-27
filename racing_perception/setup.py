from setuptools import find_packages, setup

package_name = 'racing_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ('share/racing_perception/launch', ['launch/racing_perception_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nila',
    maintainer_email='nila@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'racing_perception_node = racing_perception.perception_node:main',
        'fake_lidar_node = racing_perception.fake_lidar_node:main',
        'speed_controller_node = racing_perception.speed_controller_node:main',
'data_logger_node = racing_perception.data_logger_node:main',
        ],
    },
)
