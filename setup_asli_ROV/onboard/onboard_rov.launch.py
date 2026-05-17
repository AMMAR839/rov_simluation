from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_dry_run = LaunchConfiguration("hardware_dry_run")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_vision = LaunchConfiguration("use_vision")
    qr_image_topic = LaunchConfiguration("qr_image_topic")
    pwm_neutral_us = LaunchConfiguration("pwm_neutral_us")
    pwm_min_us = LaunchConfiguration("pwm_min_us")
    pwm_max_us = LaunchConfiguration("pwm_max_us")
    pwm_deadband_us = LaunchConfiguration("pwm_deadband_us")
    pwm_response_s = LaunchConfiguration("pwm_response_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_dry_run", default_value="true"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("use_vision", default_value="false"),
            DeclareLaunchArgument("qr_image_topic", default_value="/rov/camera/wall/image"),
            DeclareLaunchArgument("pwm_neutral_us", default_value="1500.0"),
            DeclareLaunchArgument("pwm_min_us", default_value="1400.0"),
            DeclareLaunchArgument("pwm_max_us", default_value="1600.0"),
            DeclareLaunchArgument("pwm_deadband_us", default_value="25.0"),
            DeclareLaunchArgument("pwm_response_s", default_value="0.15"),
            Node(
                package="rov_gamantaray_control",
                executable="cmd_vel_mux",
                name="onboard_cmd_vel_mux",
                output="screen",
                parameters=[{"default_source": "manual", "command_timeout_s": 0.5}],
            ),
            Node(
                package="rov_gamantaray_control",
                executable="thruster_allocator",
                name="onboard_thruster_allocator",
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
                name="onboard_serial_pwm_driver",
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
                package="rov_gamantaray_vision",
                executable="qr_detector",
                name="onboard_qr_detector",
                output="screen",
                condition=IfCondition(use_vision),
                parameters=[{"image_topic": qr_image_topic}],
            ),
        ]
    )
