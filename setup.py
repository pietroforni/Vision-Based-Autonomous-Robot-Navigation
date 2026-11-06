from setuptools import setup
from glob import glob

package_name = 'robomaster_example'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', glob('launch/*.launch'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elia Cereda',
    maintainer_email='eliacereda@gmail.com',
    description='Overhead vision navigation with OpenCV and A* for a RoboMaster',
    license='MIT',
    entry_points={
        'console_scripts': [
            'robomaster_controller_node = robomaster_example.robomaster_controller_node:main',
            'vision_demo = robomaster_example.demo:main'
        ],
    },
)
