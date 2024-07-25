import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andyv',
    maintainer_email='andy.vince2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "simple_nav = nav_pkg.simple_nav:main",
            "nav_step = nav_pkg.nav_step:main",
            "naive_obj_avoidance = nav_pkg.naive_obstacle_avoidance:main",
            "odom = nav_pkg.odom:main",
            "nav1 = nav_pkg.motion_model.nav1:main",
            "nav2 = nav_pkg.motion_model.nav2:main",
            "nav_stack = nav_pkg.motion_model.nav_stack:main",
            "navigation = nav_pkg.navigation:main",
        ],
    },
)
