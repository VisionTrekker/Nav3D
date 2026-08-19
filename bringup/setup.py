from setuptools import setup

package_name = 'bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    py_modules=['launch_helpers'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['bringup_nav3d.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/nav3d_bag.rviz']),
    ],
    install_requires=['setuptools', 'launch', 'launch_ros'],
)
