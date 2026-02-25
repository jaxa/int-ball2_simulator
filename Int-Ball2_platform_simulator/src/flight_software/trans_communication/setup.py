from setuptools import setup
import os
from glob import glob

package_name = 'trans_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JAXA',
    maintainer_email='jaxa.doe@example.com',
    description='The trans_communication package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trans_communication_node = trans_communication.trans_communication_node:main',
        ],
    },
)
