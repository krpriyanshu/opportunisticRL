from setuptools import find_packages, setup

package_name = "marine_state_estimator"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/state_estimator.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="opportunity_rl contributors",
    maintainer_email="noreply@example.com",
    description=(
        "Milestone M2: vessel/robot state pipeline (odometry, filtered joint "
        "state, vessel-relative end-effector pose)."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "state_estimator_node = marine_state_estimator.state_estimator_node:main",
        ],
    },
)
