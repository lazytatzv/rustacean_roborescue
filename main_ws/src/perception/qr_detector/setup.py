import os
from glob import glob

from setuptools import setup

package_name = "qr_detector"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # WeChatQRCode model files
        (os.path.join("share", package_name, "models"), glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lazytatzv",
    maintainer_email="yano.tatsuki439@mail.kyutech.jp",
    description="QR code detector using OpenCV WeChatQRCode",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "qr_detector_node = qr_detector.qr_detector_node:main",
        ],
    },
)
