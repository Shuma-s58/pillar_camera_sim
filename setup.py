from setuptools import setup
from glob import glob
import os

package_name = 'pillar_camera_sim'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simulation Maintainer',
    maintainer_email='maintainer@example.com',
    description='Independent keyboard-controlled virtual camera and pillar scene',
    license='MIT',
    entry_points={'console_scripts': [
        'simulation_node = pillar_camera_sim.simulation_node:main',
        'keyboard_teleop = pillar_camera_sim.keyboard_teleop:main',
    ]},
)
