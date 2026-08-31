"""Setuptools configuration for rover_safety."""

from setuptools import find_packages, setup


package_name = 'rover_safety'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Husky Robotics',
    maintainer_email='jzhou44@cs.washington.edu',
    description='Rover-side motion watchdog and emergency-stop gate',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'rover_watchdog = rover_safety.watchdog_node:main',
        ],
    },
)
