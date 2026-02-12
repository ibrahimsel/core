from setuptools import setup
import os
from glob import glob

package_name = 'muto_core'

setup(
    name=package_name,
    version='0.42.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alp SARICA',
    maintainer_email='alp.sarica@eteration.com',
    description='Eclipse Muto Core - digital twin bridge that manages device registration, connectivity, telemetry, and stack state with a remote twin server',
    license='Eclipse Public License v2.0',
    entry_points={
        'console_scripts': [
            'twin = muto_core.twin:main'
        ],
    },
)
