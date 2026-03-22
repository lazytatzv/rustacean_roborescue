from setuptools import setup

package_name = "gst_webrtc_operator"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RoboRescue",
    maintainer_email="dev@example.com",
    description="Operator-side GStreamer WebRTC ROS2 node (receiver)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "operator_webrtc_node = gst_webrtc_operator.operator_webrtc_node:main"
        ],
    },
)
