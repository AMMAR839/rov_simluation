from setuptools import find_packages, setup

package_name = "rov_gamantaray_control"

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
    description="Control nodes for a KKI-style ROV Gazebo simulation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "thruster_allocator = rov_gamantaray_control.thruster_allocator:main",
            "cmd_vel_mux = rov_gamantaray_control.cmd_vel_mux:main",
            "kinematic_driver = rov_gamantaray_control.kinematic_driver:main",
            "hydro_wrench_driver = rov_gamantaray_control.hydro_wrench_driver:main",
            "gripper_manager = rov_gamantaray_control.gripper_manager:main",
            "water_effects_driver = rov_gamantaray_control.water_effects_driver:main",
            "mission_supervisor = rov_gamantaray_control.mission_supervisor:main",
            "rov_teleop_keyboard = rov_gamantaray_control.teleop_keyboard:main",
            "rov_joystick = rov_gamantaray_control.joystick_driver:main",
        ],
    },
)
