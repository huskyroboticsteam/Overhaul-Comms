"""Setuptools configuration for mc_bridge."""

from setuptools import find_packages, setup


package_name = 'mc_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'websockets==14.2',
    ],
    zip_safe=True,
    maintainer='Husky Robotics',
    maintainer_email='jzhou44@cs.washington.edu',
    description=(
        'Communication bridge for Mission Control and rover ROS 2 nodes'
    ),
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mc_bridge = mc_bridge.main:main',
        ],
    },
)
