from setuptools import find_packages, setup
from glob import glob

package_name = 'viewer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/launch', glob('launch/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sotaro',
    maintainer_email='sotaro0416@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'viewer = viewer.viewer:main',
        ],
    },
)
