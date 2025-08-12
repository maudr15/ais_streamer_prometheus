from setuptools import find_packages, setup

package_name = 'ais_stream_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    py_modules=['ais_stream_publisher'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ntnu',
    maintainer_email='m.audrain15@gmx.fr',
    description='ROS 2 node streaming AIS data from BarentsWatch',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ais_stream_publisher = ais_stream_publisher:main'
        ],
    },
)
