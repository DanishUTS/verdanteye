from setuptools import setup
from glob import glob
import os

package_name = 've_behaviours'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', [
            'launch/auto_wander_depth.launch.py',
            'launch/x3.launch.py',
            'launch/wander_launch.py',]),

        ('share/' + package_name + '/assets', [
            've_behaviours/assets/VerdantEye.png',
        ]),
        
        # URDF / Xacro
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf') + glob('urdf/*.xacro') + glob('urdf/*.urdf.xacro')),

        # URDF meshes
        (os.path.join('share', package_name, 'urdf', 'meshes'),
         glob('urdf/meshes/*')),

        (os.path.join('share', package_name, 'config'),
         glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gohan',
    maintainer_email='gohan@todo.todo',
    description='VerdantEye Husky control UI and autonomous depth navigation scripts',
    license='MIT',
    entry_points={
        'console_scripts': [
            'auto_wander_depth = ve_behaviours.auto_wander_depth:main',
            'ui = ve_behaviours.ui:main',
            'wander_node = ve_behaviours.wander_node:main',
        ],
    },
)
