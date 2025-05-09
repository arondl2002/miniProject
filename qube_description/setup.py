from setuptools import find_packages, setup
import os

package_name = 'qube_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/view_qube.launch.py']),
        (os.path.join('share', package_name, 'urdf'), ['urdf/qube.urdf.xacro', 'urdf/qube.macro.xacro']),
        (os.path.join('share', package_name, 'rviz'), ['rviz/config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonamoll',
    maintainer_email='jonamoll@stud.ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
