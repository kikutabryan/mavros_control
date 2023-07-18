from setuptools import setup

package_name = 'mavros_control'

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
    maintainer='bryan',
    maintainer_email='kikutabryan@gmail.com',
    description='Control of a drone using MAVROS over ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_control = mavros_control.drone_control:main',
        ],
    },
)
