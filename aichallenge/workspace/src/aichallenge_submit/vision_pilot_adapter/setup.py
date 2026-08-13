from setuptools import find_packages, setup

package_name = "vision_pilot_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AI Challenge Team",
    maintainer_email="aichallenge@example.com",
    description="Autoware message adapter for Vision Pilot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vision_pilot_adapter_node = vision_pilot_adapter.node:main",
        ],
    },
)
