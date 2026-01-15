from setuptools import find_packages, setup
import os
from glob import glob

package_name = "piper_evaluation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include launch files
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="harshrocks",
    maintainer_email="harshchhajed30@gmail.com",
    description="Independent evaluation node for the Piper robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "piper_evaluator_node = piper_evaluation.piper_evaluator_node:main",
            "piper_recorder_node = piper_evaluation.piper_recorder_node:main",
            "piper_player_node = piper_evaluation.piper_player_node:main",
            "websocket_bridge_node = piper_evaluation.websocket_bridge_node:main",
            "piper_research_logger_node = piper_evaluation.piper_research_logger_node:main",
        ],
    },
)
