from setuptools import setup

package_name = 'robo_map_exporter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='you@example.com',
    description='RoboCup mapping exporter (PLY/CSV) PoC',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robo_map_exporter_node = robo_map_exporter.robo_map_exporter_node:main'
        ],
    },
)
