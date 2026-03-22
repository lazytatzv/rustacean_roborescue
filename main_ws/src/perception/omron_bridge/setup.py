from setuptools import setup

package_name = 'omron_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='you@example.com',
    description='Bridge node for OMRON 2JCIE-BU01',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'omron_bridge_node = omron_bridge.omron_bridge_node:main'
        ],
    },
)
