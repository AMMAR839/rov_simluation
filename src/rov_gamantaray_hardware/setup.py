from setuptools import find_packages, setup

package_name = "rov_gamantaray_hardware"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ammar",
    maintainer_email="ammar@example.com",
    description="Hardware interface nodes for the Gamantara ROV.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "serial_pwm_driver = rov_gamantaray_hardware.serial_pwm_driver:main",
        ],
    },
)
