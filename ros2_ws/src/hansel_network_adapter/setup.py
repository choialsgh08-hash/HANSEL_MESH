from setuptools import find_packages, setup


package_name = "hansel_network_adapter"

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
    description="Stable network adapter boundary for external team code.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "network_adapter = hansel_network_adapter.network_adapter_node:main",
        ],
    },
)

