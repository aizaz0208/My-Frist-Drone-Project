from setuptools import find_packages, setup

package_name = 'my_first_drone'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
data_files=[
    (
        'share/ament_index/resource_index/packages',
        ['resource/my_first_drone'],
    ),
    (
        'share/my_first_drone',
        ['package.xml'],
    ),
    (
        'share/my_first_drone/launch',
        ['launch/drone.launch.py'],
    ),
    (
        'share/my_first_drone/urdf',
        ['urdf/x500.urdf.xacro'],
    ),
    (
        'share/my_first_drone/config',
        ['config/px4_config.yaml'],
    ),
],    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aizaz',
    maintainer_email='aizaz@localhost',
    description='ROS 2 Jazzy keyboard control for PX4 v1.17 Gazebo Harmonic X500 via MAVLink.',
    license='Apache-2.0',
entry_points={
    'console_scripts': [
        'my_first_drone = my_first_drone.main:main',
        'mobile_controller = my_first_drone.mobile_controller:main',
    ],
},
)
