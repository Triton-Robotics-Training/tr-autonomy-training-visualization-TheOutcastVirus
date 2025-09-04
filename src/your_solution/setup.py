from setuptools import find_packages, setup

package_name = "your_solution"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/full_system_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vkommera",
    maintainer_email="vikram.kommera@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tf_broadcaster = your_solution.tf_broadcaster:main",
            # TODO FIX THE ENTERY POINT / code in the entry point
            "calc_error = calc_error.calc_error:main",
        ],
    },
)
