from glob import glob
import os


def regular_files(pattern):
    """Return only regular files matched by a data-file glob."""
    return [path for path in glob(pattern) if os.path.isfile(path)]

from setuptools import find_packages, setup


package_name = "hansel_description"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "urdf"), regular_files("urdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HANSEL Team",
    maintainer_email="hansel@example.com",
    description="HANSEL URDF and commanded joint state publisher.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "commanded_joint_state_publisher = "
            "hansel_description.commanded_joint_state_publisher:main",
        ],
    },
)

