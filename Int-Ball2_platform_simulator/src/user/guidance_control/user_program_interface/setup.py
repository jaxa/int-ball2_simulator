from setuptools import setup

package_name = 'user_program_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/user_program_interface.launch',
            'launch/user_program_interface.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JAXA',
    maintainer_email='jaxa.doe@example.com',
    description='The user_program_interface package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'user_program_interface = user_program_interface.user_program_interface_node:main',
        ],
    },
)
