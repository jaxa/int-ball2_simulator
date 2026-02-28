from setuptools import setup
import os
from glob import glob

package_name = 'platform_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JAXA',
    maintainer_email='jaxa.doe@example.com',
    description='The platform_monitor package',
    license='TODO',
    entry_points={
        'console_scripts': [
            'platform_monitor = platform_monitor.platform_monitor_node:main',
        ],
    },
)
