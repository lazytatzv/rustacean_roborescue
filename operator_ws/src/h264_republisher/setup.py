from setuptools import setup

package_name = "h264_republisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "av", "opencv-python"],
    zip_safe=True,
    maintainer="RoboRescue",
    maintainer_email="dev@example.com",
    description="Republish H.264 compressed topic as sensor_msgs/Image",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "h264_republisher = h264_republisher.h264_republisher:main"
        ],
    },
)
