from setuptools import find_packages, setup

package_name = 'zeuscar_robot_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Murasan201',
    maintainer_email='shinf0330@gmail.com',
    description='ZeusCar robot control package for Arduino communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber_node = zeuscar_robot_package.subscriber:main',
        ],
    },
)
