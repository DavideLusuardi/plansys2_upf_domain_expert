from setuptools import setup

package_name = 'plansys2_upf_domain_expert'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davide',
    maintainer_email='davide.lusuardi@studenti.unitn.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = plansys2_upf_domain_expert.domain_expert_node:main',
            'client = plansys2_upf_domain_expert.my_client:main',
        ],
    },
)
