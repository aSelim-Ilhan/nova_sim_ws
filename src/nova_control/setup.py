from setuptools import find_packages, setup

package_name = 'nova_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='selim',
    maintainer_email='tcahmet.selim@gmail.com',
    description='Thruster allocation and PID controllers for NOVA AUV',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'thruster_allocator  = nova_control.thruster_allocator:main',
            'pid_controller      = nova_control.pid_controller:main',
            'cmd_vel_to_wrench   = nova_control.cmd_vel_to_wrench:main',
            'gazebo_bridge       = nova_control.gazebo_bridge:main',
            'hydrodynamics       = nova_control.hydrodynamics:main',
            'teleop              = nova_control.teleop:main',
        ],
    },
)
