from setuptools import setup

package_name = 've_behaviours'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/auto_wander_depth.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Danish Silva',
    maintainer_email='you@example.com',
    description='Behaviour nodes (wander, UI helpers, etc.)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'auto_wander_depth = ve_behaviours.auto_wander_depth:main',
        ],
    },
)
