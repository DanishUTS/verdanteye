# file: src/ve_behaviours/setup.py
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

        # ✅ bamboo + other launch files
        ('share/' + package_name + '/launch', [
            'launch/auto_wander_depth.launch.py',
            'launch/x3.launch.py',
            'launch/wander_launch.py',
            'launch/bamboo_randomizer.launch.py',
        ]),

        ('share/' + package_name + '/assets', [
            'assets/VerdantEye.png',
        ]),

        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf') + glob('urdf/*.xacro') + glob('urdf/*.urdf.xacro')),

        (os.path.join('share', package_name, 'urdf', 'meshes'),
         glob('urdf/meshes/*')),

        (os.path.join('share', package_name, 'config'),
         glob('config/*')),

        (os.path.join('share', package_name, 'maps'),
         glob('maps/*')),

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
            'bamboo_randomizer = ve_behaviours.bamboo_randomizer:main',

            # NEW: separate checklist/gallery UI (only if you keep ui_plants.py)
            'ui_plants = ve_behaviours.ui_plants:main',
            'maze_wander_node = ve_behaviours.maze_wander_node:main',
        ],
    },
)
