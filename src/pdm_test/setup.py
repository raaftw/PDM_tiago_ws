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

        # install Gazebo worlds
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.world')),

        # install maps (located one level up at ../maps)
        (os.path.join('share', package_name, 'maps'),
         glob(os.path.join('..', 'maps', '*'))),

        # install launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML', 'Pillow'],
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
            'straight_line_planner = pdm_test.global_planner_straight:main',
            'rrt_star_planner = pdm_test.rrt_star_planner:main',
            'simple_map_publisher = pdm_test.simple_map_publisher:main',
            'mpc_controller = pdm_test.mpc_controller:main'
        ],
    },
)
