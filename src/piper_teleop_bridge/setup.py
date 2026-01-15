from setuptools import find_packages, setup

package_name = "piper_teleop_bridge"

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
    maintainer="harshrocks",
    maintainer_email="harshchhajed30@gmail.com",
    description="A clean websocket bridge for Piper teleoperation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Client Node (Node 3)
            "websocket_bridge_3 = piper_teleop_bridge.websocket_bridge_node3:main",
            # Server Node (Node 4)
            "websocket_bridge_4 = piper_teleop_bridge.websocket_bridge_node4:main",
        ],
    },
)
