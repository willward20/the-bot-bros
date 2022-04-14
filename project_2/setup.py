import os
from setuptools import setup
from glob import glob

package_name = 'project_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join("launch", "*.py"))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join("urdf", "*.urdf"))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join("config", "*.yaml"))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics-b',
    maintainer_email='willward1912@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher = project_2.odom_publisher:main'
        ],
    },
)
