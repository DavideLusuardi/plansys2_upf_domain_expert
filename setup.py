import os
from glob import glob
from setuptools import setup

package_name = 'plansys2_upf_domain_expert'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        
        # Include test files
        (os.path.join('share', package_name, 'test', 'pddl'), glob('test/pddl/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davide',
    maintainer_email='davide.lusuardi@studenti.unitn.it',
    description='This package contains the Domain Expert module for the ROS2 Planning System based on the Unified Planning Framework',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'domain_expert_node = plansys2_upf_domain_expert.domain_expert_node:main',
        ],
    },
)
