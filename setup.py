from setuptools import setup
import os
from glob import glob

package_name = 'urad'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='Nicolas Sarmiento',
    maintainer_email='nicosarmientom@gmail.com',
    description='Basic Example nodes for uRAD radar',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'urad_publisher = urad.urad_publisher:main',
            'urad_publisher_continuous = urad.urad_publisher_continuous:main'
            'urad_dummy_publisher = urad.dummy_radar_publisher:main',
            'urad_subscriber = urads.subscriber:main',
            'urad_zero_ff = urad.zero_ff_node:main',
        ],
    },
)
