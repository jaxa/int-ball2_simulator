import os
from glob import glob
from setuptools import setup

package_name = 'sample_tests'

setup(
    name=package_name,
    version='0.9.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JAXA',
    maintainer_email='jaxa.doe@example.com',
    description='The sample_tests package',
    license='N/A',
    entry_points={
        'console_scripts': [
            'simple_test = sample_tests.simple_test:main',
            'test_camera_images = sample_tests.test_camera_images:main',
            'test_fan_duty = sample_tests.test_fan_duty:main',
            'test_force_and_torque = sample_tests.test_force_and_torque:main',
            'test_force_torque_and_fan_duty = sample_tests.test_force_torque_and_fan_duty:main',
            'test_led_colors = sample_tests.test_led_colors:main',
            'test_navigation = sample_tests.test_navigation:main',
            'test_visual_slam = sample_tests.test_visual_slam:main',
        ],
    },
)
