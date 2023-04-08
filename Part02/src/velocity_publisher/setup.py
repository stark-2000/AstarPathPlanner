from setuptools import setup

package_name = 'velocity_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhinesh',
    maintainer_email='dhineshrajasekaran@gmail.com',
    description='Astar Implementation using Turtlebot3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'vel_pub = velocity_publisher.vel_pub:main'
        ],
    },
)
