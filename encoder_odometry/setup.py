from setuptools import setup

package_name = 'encoder_odometry'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoshi',
    maintainer_email='yoshi@todo.todo',
    description='Encoder odometry node for RC car (replaces VESC odometry)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_odom_node = encoder_odometry.encoder_odom_node:main',
        ],
    },
)
