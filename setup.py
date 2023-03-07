from setuptools import setup

package_name = 'urad'

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
    maintainer='lab',
    maintainer_email='nis3770@thi.de',
    description='Example nodes for urad radar',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'publisher = urad.publisher:main', 'subscriber = urad.subscriber:main',
		'test_pub = urad.test_pub:main', 'test_sub = urad.test_sub:main',
        'urad_test = urad.test:main', 'zero_ff_node = urad.zero_ff_node:main',
	'urad_pub_no_timer= urad.test_pub_no_timer:main'
        ],
    },
)
