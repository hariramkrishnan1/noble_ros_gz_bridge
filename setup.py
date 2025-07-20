from setuptools import setup

package_name = 'noble_ros_gz_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=['scripts.dynamic_bridge'],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/bridge_launch.py']),
        ('share/' + package_name + '/config', ['config/bridge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tony Stark',
    maintainer_email='stark@noble.ai',
    description='Advanced ROS-Gazebo bridge with YAML config, services, and actions.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dynamic_bridge.py = scripts.dynamic_bridge:main',
        ],
    },
)
