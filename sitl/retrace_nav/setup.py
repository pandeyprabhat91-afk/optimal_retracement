from setuptools import find_packages, setup

package_name = "retrace_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="retrace",
    maintainer_email="retrace@local",
    description="GPS-loss retrace estimator (C1-C6 combos)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "retrace_node = retrace_nav.retrace_node:main",
            "commander = retrace_nav.commander:main",
        ],
    },
)
