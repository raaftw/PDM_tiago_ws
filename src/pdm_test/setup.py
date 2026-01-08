from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'pdm_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        # ament index registration
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # install Gazebo worlds (if present)
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.world')),

        # install all launch files (including nested folders)
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/**/*.launch.py', recursive=True)),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/**/*.launch.xml', recursive=True)),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/**/*.py', recursive=True)),

        # install maps from package-local maps/ (recommended)
        (os.path.join('share', package_name, 'maps'),
         glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
         glob('maps/*.pgm')),
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
            'mpc_controller = pdm_test.local_planner:main',
            'obstacle_publisher = pdm_test.obstacle_publisher:main',
            'global_planner = pdm_test.global_planner:main',
            'ground_truth_republisher = pdm_test.ground_truth_republisher:main',
            'tiago_table_cleaner = pdm_test.tiago_table_cleaner:main',
            'tiago_table_cleaner_simple = pdm_test.tiago_table_cleaner_simple:main',
            'tiago_table_cleaner_fk = pdm_test.tiago_table_cleaner_fk:main',
            'tiago_table_cleaner_local_fk = pdm_test.tiago_table_cleaner_local_fk:main',
            'tiago_table_cleaner_rrt = pdm_test.tiago_table_cleaner_rrt:main',
            'table_detector = pdm_test.table_detector:main',
            'tiago_table_cleaner_rrt_ik = pdm_test.tiago_table_cleaner_rrt_ik:main',
            'ik_debug_client = pdm_test.ik_debug_client:main',
            'tiago_table_cleaner_rrt_visualization_ik = pdm_test.tiago_table_cleaner_rrt_visualization_ik:main',
        ],
    },
)

