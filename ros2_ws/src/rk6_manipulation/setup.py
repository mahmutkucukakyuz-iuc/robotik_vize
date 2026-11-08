import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rk6_manipulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mahmut',
    maintainer_email='mahmutkucukakyuz@ogr.iuc.edu.tr',
    description='rk6 ek senaryo: ters kinematik, kamera ile kup algilama, al-birak ve ust uste dizme',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ik_goto = rk6_manipulation.ik_goto:main',
            'pick_place = rk6_manipulation.pick_place:main',
            'scan_and_stack = rk6_manipulation.scan_and_stack:main',
        ],
    },
)
