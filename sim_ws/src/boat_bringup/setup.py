
from setuptools import setup

package_name = 'boat_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        ('share/' + package_name + '/config', ['boat_bringup/config/bridge.yaml', 'boat_bringup/config/rviz.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aquatonomous',
    maintainer_email='noreply@example.com',
    description='Bringup and launch files for aQuatonomous sim',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
