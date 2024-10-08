from setuptools import find_packages, setup

package_name = 'cometbot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amadorjosephg',
    maintainer_email='amadorjosephg@gmail.com',
    description='Control Package for test lunabot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'teleop_publisher = cometbot_control.teleop_publisher:main'
        ],
    },
)
