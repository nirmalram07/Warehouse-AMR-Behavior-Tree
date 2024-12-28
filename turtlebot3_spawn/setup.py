from setuptools import find_packages, setup

package_name = 'turtlebot3_spawn'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtlebot_gazebo.launch.py']),
        ('share/' + package_name + '/map', ['map/factory_map.yaml','map/factory_map.pgm']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']) 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nirmal',
    maintainer_email='nirmalcgvfx@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amcl_pose_intializer = turtlebot3_spawn.set_init_pose:main'
        ],
    },
)
