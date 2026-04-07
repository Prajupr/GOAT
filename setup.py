from setuptools import find_packages, setup

package_name = 'butler_delivery'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/butler_delivery.launch.py',
            'launch/visualization_only.launch.py',
            'launch/spawn_turtlebot.launch.py'
        ]),
        ('share/' + package_name + '/config', ['config/delivery_visualization.rviz', 'config/restaurant_nav2_params.yaml', 'config/butler_rviz.rviz']),
        ('share/' + package_name + '/models', ['models/table.sdf', 'models/kitchen.sdf']),
        ('share/' + package_name + '/worlds', ['worlds/restaurant.world']),
        ('share/' + package_name + '/maps', ['maps/restaurant.pgm', 'maps/restaurant.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='praju',
    maintainer_email='praju@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'delivery_manager = butler_delivery.delivery_manager:main',
            'order_publisher = butler_delivery.order_publisher:main',
            'interactive_client = butler_delivery.interactive_client:main',
            'scenario_runner = butler_delivery.scenario_runner:main',
            'robot_visualizer = butler_delivery.robot_visualizer:main',
        ],
    },
)
