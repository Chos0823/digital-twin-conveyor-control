from setuptools import find_packages, setup

package_name = "conveyor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="noosgnoy",
    maintainer_email="noosgnoy@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "con = conveyor.conveyor_node:main",
            "gui = conveyor.conveyor_gui_node:main",
        ],
    },
)
