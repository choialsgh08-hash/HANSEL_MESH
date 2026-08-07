from setuptools import find_packages, setup


package_name = "hansel_operator"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "plugin.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HANSEL Team",
    maintainer_email="hansel@example.com",
    description="HANSEL operator input, routing, detach coordination, and RQT.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "command_router = hansel_operator.command_router:main",
            "detach_coordinator = hansel_operator.detach_coordinator:main",
            "event_logger = hansel_operator.event_logger:main",
            "operator_input = hansel_operator.operator_input:main",
        ],
    },
)

