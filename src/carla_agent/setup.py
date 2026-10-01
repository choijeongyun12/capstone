from setuptools import find_packages, setup

package_name = 'carla_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/carla_agent.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tmo',
    maintainer_email='seamekorea@gmail.com',
    description='Carla ego vehicle spawner with front/rear cameras and radar-based autopilot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vehicle_spawner = carla_agent.vehicle_spawner:main',
            'autopilot_node  = carla_agent.autopilot_node:main',
            'lane_follower   = carla_agent.lane_follower:main',
            'visualizer          = carla_agent.visualizer:main',
            'spectator_follower  = carla_agent.spectator_follower:main',
            'object_detector     = carla_agent.object_detector:main',
        ],
    },
)
