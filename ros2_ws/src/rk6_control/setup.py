import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rk6_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mahmut',
    maintainer_email='mahmutkucukakyuz@ogr.iuc.edu.tr',
    description='rk6 robot kol: EE pozu, hareket dizisi, gorsellestirme ve FK hata analizi',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ee_pose_publisher = rk6_control.ee_pose_publisher:main',
            'motion_sequence = rk6_control.motion_sequence:main',
            'visualizer = rk6_control.visualizer:main',
            'fk_error_analysis = rk6_control.fk_error_analysis:main',
        ],
    },
)
