from setuptools import find_packages, setup


package_name = "hansel_radar_adapter"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HANSEL Team",
    maintainer_email="hansel@example.com",
    description="Stable PointCloud/Map/TF boundary for external Radar code.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "radar_adapter = hansel_radar_adapter.radar_adapter_node:main",
        ],
    },
)

