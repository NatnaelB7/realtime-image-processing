from setuptools import find_packages, setup

package_name = 'orange'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/webcam_color_filter.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='natnael',
    maintainer_email='nbtakele@aggies.ncat.edu',
    description='Webcam driver with timestamp',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'webcam_driver = orange.webcam_driver:main',
        ],
    },
)

