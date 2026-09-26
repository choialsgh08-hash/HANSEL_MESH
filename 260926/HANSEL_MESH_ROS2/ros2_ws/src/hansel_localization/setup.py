from setuptools import find_packages, setup


package_name = "hansel_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HANSEL Team",
    maintainer_email="hansel@example.com",
    description="BNO085 and rear-encoder planar odometry for HANSEL Head.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "head_odometry = hansel_localization.head_odometry_node:main",
        ],
    },
)
