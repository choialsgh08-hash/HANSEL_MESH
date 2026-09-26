from glob import glob
import os


def regular_files(pattern):
    """Return only regular files matched by a data-file glob."""
    return [path for path in glob(pattern) if os.path.isfile(path)]

from setuptools import find_packages, setup


package_name = "hansel_bringup"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), regular_files("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), regular_files("config/*.yaml")),
        (
            os.path.join("share", package_name, "config", "dds"),
            regular_files("config/dds/*"),
        ),
        (os.path.join("share", package_name, "rviz"), regular_files("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HANSEL Team",
    maintainer_email="hansel@example.com",
    description="HANSEL launch/configuration package.",
    license="MIT",
)

