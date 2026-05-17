from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joystick = LaunchConfiguration("joystick")
    joystick_device = LaunchConfiguration("joystick_device")
    joystick_enable_button = LaunchConfiguration("joystick_enable_button")
    use_vision = LaunchConfiguration("use_vision")
    kki_gui = LaunchConfiguration("kki_gui")
    kki_gui_window = LaunchConfiguration("kki_gui_window")
    team_name = LaunchConfiguration("team_name")
    university_name = LaunchConfiguration("university_name")
    hardware_dry_run = LaunchConfiguration("hardware_dry_run")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    pwm_neutral_us = LaunchConfiguration("pwm_neutral_us")
    pwm_min_us = LaunchConfiguration("pwm_min_us")
    pwm_max_us = LaunchConfiguration("pwm_max_us")
    pwm_deadband_us = LaunchConfiguration("pwm_deadband_us")
    pwm_response_s = LaunchConfiguration("pwm_response_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument("joystick", default_value="true"),
            DeclareLaunchArgument("joystick_device", default_value="/dev/input/js0"),
            DeclareLaunchArgument("joystick_enable_button", default_value="4"),
            DeclareLaunchArgument("use_vision", default_value="false"),
            DeclareLaunchArgument("kki_gui", default_value="true"),
            DeclareLaunchArgument("kki_gui_window", default_value="true"),
            DeclareLaunchArgument("team_name", default_value="Gamantara"),
            DeclareLaunchArgument("university_name", default_value="Universitas Gadjah Mada"),
            DeclareLaunchArgument("hardware_dry_run", default_value="true"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("pwm_neutral_us", default_value="1500.0"),
            DeclareLaunchArgument("pwm_min_us", default_value="1400.0"),
            DeclareLaunchArgument("pwm_max_us", default_value="1600.0"),
            DeclareLaunchArgument("pwm_deadband_us", default_value="25.0"),
            DeclareLaunchArgument("pwm_response_s", default_value="0.15"),
            Node(
                package="rov_gamantaray_control",
                executable="cmd_vel_mux",
                name="cmd_vel_mux",
                output="screen",
                parameters=[
                    {
                        "default_source": "manual",
                        "command_timeout_s": 0.5,
                    }
                ],
            ),
            Node(
                package="rov_gamantaray_control",
                executable="thruster_allocator",
                name="thruster_allocator",
                output="screen",
                parameters=[
                    {
                        "pwm_neutral_us": pwm_neutral_us,
                        "pwm_min_us": pwm_min_us,
                        "pwm_max_us": pwm_max_us,
                        "pwm_deadband_us": pwm_deadband_us,
                        "pwm_response_s": pwm_response_s,
                    }
                ],
            ),
            Node(
                package="rov_gamantaray_hardware",
                executable="serial_pwm_driver",
                name="serial_pwm_driver",
                output="screen",
                parameters=[
                    {
                        "dry_run": hardware_dry_run,
                        "serial_port": serial_port,
                        "baud_rate": baud_rate,
                        "pwm_neutral_us": pwm_neutral_us,
                        "pwm_min_us": pwm_min_us,
                        "pwm_max_us": pwm_max_us,
                        "command_timeout_s": 0.5,
                    }
                ],
            ),
            Node(
                package="rov_gamantaray_control",
                executable="rov_joystick",
                name="rov_joystick",
                output="screen",
                condition=IfCondition(joystick),
                parameters=[
                    {
                        "device_path": joystick_device,
                        "enable_button": joystick_enable_button,
                        "linear_scale": 0.30,
                        "vertical_scale": 0.25,
                        "yaw_scale": 0.25,
                    }
                ],
            ),
            Node(
                package="rov_gamantaray_vision",
                executable="qr_detector",
                name="qr_detector",
                output="screen",
                condition=IfCondition(use_vision),
            ),
            Node(
                package="rov_gamantaray_control",
                executable="kki_dashboard",
                name="kki_dashboard",
                output="screen",
                condition=IfCondition(kki_gui),
                parameters=[
                    {
                        "team_name": team_name,
                        "university_name": university_name,
                        "window_enabled": kki_gui_window,
                    }
                ],
            ),
        ]
    )
