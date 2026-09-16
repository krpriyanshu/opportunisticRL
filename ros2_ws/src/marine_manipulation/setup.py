import glob

from setuptools import find_packages, setup

package_name = "marine_manipulation"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/models/grasp_target", glob.glob("models/grasp_target/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="opportunity_rl contributors",
    maintainer_email="noreply@example.com",
    description="Milestone M3: conventional grasp pipeline against a static target.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grasp_pipeline = marine_manipulation.grasp_pipeline:main",
        ],
    },
)
