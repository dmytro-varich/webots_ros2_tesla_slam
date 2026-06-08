"""webots_ros2_tesla_slam package setup file."""

from glob import glob

from setuptools import setup


package_name = 'webots_ros2_tesla_slam'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py',]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/tesla_world.wbt', 'worlds/.tesla_world.wbproj',
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/tesla_webots.urdf'
]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/config', [
    'config/nav2_params.yaml',
    'config/rviz_slam_config.rviz',
    'config/rviz_nav_config.rviz',
    'config/cartographer.lua'
]))
data_files.append(('share/' + package_name + '/maps', ['maps/city_map.yaml', 'maps/city_map.pgm']))
data_files.append(('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')))

setup(
    name=package_name,
    version='2026.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Dmytro Varich',
    author_email='varich.it@gmail.com',
    maintainer='Dmytro Varich',
    maintainer_email='varich.it@gmail.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Tesla ROS2 interface for Webots using SLAM and Nav2.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_ackermann = webots_ros2_tesla_slam.cmd_vel_to_ackermann:main',
            'lane_follower = webots_ros2_tesla_slam.lane_follower:main',
            'tesla_driver = webots_ros2_tesla_slam.tesla_driver:main',
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
