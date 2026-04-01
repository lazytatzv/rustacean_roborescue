from setuptools import find_packages, setup

package_name = "audio_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="yano.tatsuki439@mail.kyutech.jp",
    description="Bidirectional voice over ROS 2 topics (GStreamer/Opus)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "audio_sender   = audio_bridge.audio_sender_node:main",
            "audio_receiver = audio_bridge.audio_receiver_node:main",
        ],
    },
)
