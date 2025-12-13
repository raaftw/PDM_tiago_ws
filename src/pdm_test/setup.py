from setuptools import setup
from glob import glob
import os

package_name = 'pdm_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=['pdm_test'],
    data_files=[
        # ament index registration
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # install Gazebo worlds (if present)
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.world')),

        # install all launch files (py, xml) and rviz configs located in launch/
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*')),

        # install maps (yaml + pgm) from workspace src/maps into package share
        (os.path.join('share', package_name, 'maps'),
         glob(os.path.join('..', 'maps', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'),
         glob(os.path.join('..', 'maps', '*.pgm'))),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='raaf',
    maintainer_email='59016069+raaftw@users.noreply.github.com',
    description='PDM test package for TIAGo simulations with custom cafe worlds',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'straight_driver = pdm_test.straight_driver:main',
            'trajectory_generator = pdm_test.trajectory_generator:main',
            'mpc_controller = pdm_test.mpc_controller:main',
            'global_planner = pdm_test.global_planner:main'
        ],
    },
)
