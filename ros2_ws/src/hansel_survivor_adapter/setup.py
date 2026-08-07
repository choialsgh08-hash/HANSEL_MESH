from setuptools import find_packages, setup


package_name = "hansel_survivor_adapter"

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
    description="Stable survivor AP and communication event boundary.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "survivor_adapter = "
            "hansel_survivor_adapter.survivor_adapter_node:main",
        ],
    },
)

