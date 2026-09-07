from setuptools import setup


package_name = 'lio_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'kiss-icp>=1.3.0,<1.4'],
    entry_points={
        'console_scripts': [
            'auto_initialpose_once = lio_localization.auto_initialpose_once:main',
            'localization_composer = lio_localization.localization_composer:main',
            'fixed_map_icp = lio_localization.fixed_map_icp_node:main',
        ],
    },
)
