from setuptools import find_packages, setup


package_name = "hansel_camera_bridge"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/test_assets", ["test_assets/test_pattern.jpg"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HANSEL Team",
    maintainer_email="hansel@example.com",
    description="Operator-side H.264/RTP receive quality monitor.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_receiver = hansel_camera_bridge.camera_receiver_node:main",
            "dummy_camera_publisher = hansel_camera_bridge.dummy_camera_publisher:main",
            "camera_quality_monitor = "
            "hansel_camera_bridge.camera_quality_monitor:main",
        ],
    },
)
