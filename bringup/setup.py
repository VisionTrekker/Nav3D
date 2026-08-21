from setuptools import setup
import glob

package_name = 'bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('bringup/*.launch.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', ['rviz/nav3d_bag.rviz']),
    ],
    install_requires=['setuptools', 'launch', 'launch_ros'],
    entry_points={
        'console_scripts': [
            'planning_safety_supervisor.py = bringup.planning_safety_supervisor:main',
            'planning_acceptance_tester.py = bringup.planning_acceptance_tester:main',
        ],
    },
)
