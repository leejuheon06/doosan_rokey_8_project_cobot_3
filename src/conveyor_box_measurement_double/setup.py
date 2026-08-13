from setuptools import find_packages, setup


package_name = "conveyor_box_measurement"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src", exclude=("test",)),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            [
                "config/measurement.yaml",
                "config/measurement_conveyor_1.yaml",
                "config/measurement_conveyor_2.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="rokey@todo.todo",
    description="Depth-camera box measurement node for H2017 palletizing.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "conveyor_box_measurement_node = conveyor_box_measurement.node:main",
        ],
    },
)
