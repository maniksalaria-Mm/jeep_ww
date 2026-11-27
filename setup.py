import os
from glob import glob

from setuptools import setup

package_name = 'jeep_ww'


def files(pattern):
    return [f for f in glob(pattern) if os.path.isfile(f)]


setup(
    name=package_name,
    version='1.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), files('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), files('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), files('meshes/*')),
        (os.path.join('share', package_name, 'worlds'), files('worlds/*')),
        (os.path.join('share', package_name, 'config'), files('config/*')),
        (os.path.join('share', package_name, 'maps'), files('maps/*')),
        (os.path.join('share', package_name, 'textures'), files('textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manik',
    maintainer_email='manik@todo.com',
    description='Off-road SUV in a mountain world with SLAM and Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'auto_drive = jeep_ww.auto_drive:main',
            'go_to = jeep_ww.go_to:main',
            'save_map = jeep_ww.save_map:main',
            'stop_sim = jeep_ww.stop_sim:main',
        ],
    },
)
