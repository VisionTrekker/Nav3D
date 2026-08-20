from setuptools import setup

package_name = 'kiss_icp_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'kiss-icp>=1.3.0',
        'numpy',
        'open3d',
    ],
    entry_points={
        'console_scripts': [
            'kiss_icp_node = kiss_icp_bridge.kiss_icp_node:main',
        ],
    },
)
