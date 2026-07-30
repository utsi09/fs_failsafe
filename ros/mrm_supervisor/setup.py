import os
from glob import glob

from setuptools import setup

package_name = "mrm_supervisor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="taewook",
    maintainer_email="utsi09@g.skku.edu",
    description="Score-based MRM supervisor (mrm_move to shoulder/parking lot)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "supervisor = mrm_supervisor.supervisor:main",
        ],
    },
)
