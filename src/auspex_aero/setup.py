from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'auspex_aero'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'auspex_msgs',
        'auspex_aero_msgs'
        ],
    zip_safe=True,
    maintainer='Maximilian Schnell',
    maintainer_email='maximilian.schnell@unibw.de',
    description='AUSPEX-AERO - Offboard Controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'offboard_controller = auspex_aero.offboard_controller.offboard_controller:main',
            'flight_manager = auspex_aero.gnc.flight_manager:main',
            'drop_servo = auspex_aero.peripheral_devices.other.drop_servo:main',
            'rtsp_bridge = auspex_aero.peripheral_devices.cameras.camera_stream.rtsp_bridge:main',
            'rpi5_camera = auspex_aero.peripheral_devices.cameras.rpi5_camera:main',
            'sim_is_camera = auspex_aero.peripheral_devices.cameras.sim_is_camera:main',
            'sim_ue_camera = auspex_aero.peripheral_devices.cameras.sim_ue_camera:main',
        ],
    },
)
