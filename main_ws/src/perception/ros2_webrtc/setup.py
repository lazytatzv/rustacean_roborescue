from setuptools import setup

package_name = "ros2_webrtc"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools", "aiohttp", "pygobject"],
    zip_safe=True,
    maintainer="You",
    maintainer_email="you@example.com",
    description="ROS2-managed WebRTC audio components",
    license="Apache-2.0",
    classifiers=["Programming Language :: Python :: 3"],
    entry_points={
        "console_scripts": [
            "signaling_node=ros2_webrtc.signaling_node:main",
            "robot_webrtc_node=ros2_webrtc.robot_webrtc_node:main",
            "operator_webrtc_node=ros2_webrtc.operator_webrtc_node:main",
        ],
    },
)
