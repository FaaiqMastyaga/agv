from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'map_database'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][y]'))),
    ],
    install_requires=['setuptools', 'map_database_interfaces'],
    zip_safe=True,
    maintainer='faaiq',
    maintainer_email='faaiqmastyaga@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_handler_node = map_database.map_handler:main',
        ],
    },
)
