from setuptools import setup

package_name = 'pigpio_pwm_driver'

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
    description='PWM driver for RC car using pigpio (replaces VESC driver)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pwm_driver_node = pigpio_pwm_driver.pwm_driver_node:main',
        ],
    },
)
