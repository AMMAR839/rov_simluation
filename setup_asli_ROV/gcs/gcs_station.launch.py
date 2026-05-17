from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joystick_device = LaunchConfiguration("joystick_device")
    joystick_enable_button = LaunchConfiguration("joystick_enable_button")
    joystick_linear_scale = LaunchConfiguration("joystick_linear_scale")
    joystick_vertical_scale = LaunchConfiguration("joystick_vertical_scale")
    joystick_yaw_scale = LaunchConfiguration("joystick_yaw_scale")
    use_vision = LaunchConfiguration("use_vision")
    qr_image_topic = LaunchConfiguration("qr_image_topic")
    kki_gui = LaunchConfiguration("kki_gui")

    return LaunchDescription(
        [
            DeclareLaunchArgument("joystick_device", default_value="/dev/input/js0"),
            DeclareLaunchArgument("joystick_enable_button", default_value="4"),
            DeclareLaunchArgument("joystick_linear_scale", default_value="0.30"),
            DeclareLaunchArgument("joystick_vertical_scale", default_value="0.25"),
            DeclareLaunchArgument("joystick_yaw_scale", default_value="0.25"),
            DeclareLaunchArgument("use_vision", default_value="false"),
            DeclareLaunchArgument("qr_image_topic", default_value="/rov/camera/wall/image"),
            DeclareLaunchArgument("kki_gui", default_value="true"),
            Node(
                package="rov_gamantaray_control",
                executable="rov_joystick",
                name="gcs_rov_joystick",
                output="screen",
                parameters=[
                    {
                        "device_path": joystick_device,
                        "enable_button": joystick_enable_button,
                        "linear_scale": joystick_linear_scale,
                        "vertical_scale": joystick_vertical_scale,
                        "yaw_scale": joystick_yaw_scale,
                    }
                ],
            ),
            Node(
                package="rov_gamantaray_control",
                executable="kki_dashboard",
                name="gcs_kki_dashboard",
                output="screen",
                condition=IfCondition(kki_gui),
            ),
            Node(
                package="rov_gamantaray_vision",
                executable="qr_detector",
                name="gcs_qr_detector",
                output="screen",
                condition=IfCondition(use_vision),
                parameters=[{"image_topic": qr_image_topic}],
            ),
        ]
    )
