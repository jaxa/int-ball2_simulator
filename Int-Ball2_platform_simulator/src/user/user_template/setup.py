from setuptools import setup

package_name = 'user_template'

setup(
    name=package_name,
    version='0.9.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/template_py.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JAXA',
    maintainer_email='jaxa.doe@example.com',
    description='The user_template package',
    license='N/A',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'user_template_node = user_template.user_template_node:main',
        ],
    },
)
