import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


VALID_PAYLOADS = {"A", "B", "C", "D"}


def launch_setup(context, *args, **kwargs):
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    use_vision = LaunchConfiguration("use_vision")
    qr_image_topic = LaunchConfiguration("qr_image_topic")
    mission_autonomy = LaunchConfiguration("mission_autonomy")
    mission_autonomy_enabled = mission_autonomy.perform(context).lower() == "true"
    mission_profile = LaunchConfiguration("mission_profile")
    auto_start_on_attached = LaunchConfiguration("auto_start_on_attached")
    command_source_arg = LaunchConfiguration("command_source").perform(context).lower()
    joystick = LaunchConfiguration("joystick")
    joystick_device = LaunchConfiguration("joystick_device")
    joystick_axis_surge = LaunchConfiguration("joystick_axis_surge")
    joystick_axis_sway = LaunchConfiguration("joystick_axis_sway")
    joystick_axis_heave = LaunchConfiguration("joystick_axis_heave")
    joystick_axis_yaw = LaunchConfiguration("joystick_axis_yaw")
    joystick_invert_surge = LaunchConfiguration("joystick_invert_surge")
    joystick_invert_sway = LaunchConfiguration("joystick_invert_sway")
    joystick_invert_heave = LaunchConfiguration("joystick_invert_heave")
    joystick_invert_yaw = LaunchConfiguration("joystick_invert_yaw")
    joystick_deadzone = LaunchConfiguration("joystick_deadzone")
    joystick_linear_scale = LaunchConfiguration("joystick_linear_scale")
    joystick_vertical_scale = LaunchConfiguration("joystick_vertical_scale")
    joystick_yaw_scale = LaunchConfiguration("joystick_yaw_scale")
    joystick_enable_button = LaunchConfiguration("joystick_enable_button")
    hydro_horizontal_force_gain = LaunchConfiguration("hydro_horizontal_force_gain")
    hydro_vertical_force_gain = LaunchConfiguration("hydro_vertical_force_gain")
    hydro_yaw_torque_gain = LaunchConfiguration("hydro_yaw_torque_gain")
    hanging_constraint_enabled = LaunchConfiguration("hanging_constraint_enabled")
    hanging_damping = LaunchConfiguration("hanging_damping")
    hanging_release_velocity_gain = LaunchConfiguration("hanging_release_velocity_gain")
    hanging_max_angle_rad = LaunchConfiguration("hanging_max_angle_rad")
    physics_mode = LaunchConfiguration("physics_mode").perform(context).lower()
    hydro_control_mode = LaunchConfiguration("hydro_control_mode").perform(context).lower()
    rov_variant = LaunchConfiguration("rov_variant").perform(context).lower()
    payload_code = LaunchConfiguration("payload_code").perform(context).upper()
    if payload_code not in VALID_PAYLOADS:
        raise RuntimeError("payload_code must be one of A, B, C, or D")
    if physics_mode not in {"kinematic", "hydro"}:
        raise RuntimeError("physics_mode must be kinematic or hydro")
    if hydro_control_mode not in {"kinematic", "wrench"}:
        raise RuntimeError("hydro_control_mode must be kinematic or wrench")
    if rov_variant not in {"github_blue", "bluerov"}:
        raise RuntimeError("rov_variant must be github_blue or bluerov")
    if command_source_arg == "auto_if_mission":
        command_source = "auto" if mission_autonomy_enabled else "manual"
    elif command_source_arg in {"manual", "auto"}:
        command_source = command_source_arg
    else:
        raise RuntimeError("command_source must be manual, auto, or auto_if_mission")

    desc_share = get_package_share_directory("rov_gamantaray_description")
    gazebo_share = get_package_share_directory("rov_gamantaray_gazebo")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    template_path = Path(gazebo_share) / "worlds" / "kki_rov_pool.template.sdf"
    generated_world = Path("/tmp") / f"kki_rov_pool_{payload_code}_{physics_mode}.sdf"
    prop_layout = "none"
    if physics_mode == "hydro":
        rov_model_uri = "gamantaray_rov_hydro"
        left_gripper_jaw_uri = "gamantaray_gripper_ricketts_left_jaw_visual"
        right_gripper_jaw_uri = "gamantaray_gripper_ricketts_right_jaw_visual"
    elif rov_variant == "github_blue":
        rov_model_uri = "gamantaray_rov_github_blue_gripper"
        left_gripper_jaw_uri = "gamantaray_gripper_ricketts_left_jaw_visual"
        right_gripper_jaw_uri = "gamantaray_gripper_ricketts_right_jaw_visual"
        prop_layout = "github_blue"
    else:
        rov_model_uri = "gamantaray_rov"
        left_gripper_jaw_uri = "gamantaray_gripper_left_jaw_visual"
        right_gripper_jaw_uri = "gamantaray_gripper_right_jaw_visual"
        prop_layout = "legacy_bluerov"
    physics_step_size = "0.001" if physics_mode == "hydro" else "0.005"
    hydro_world_plugins = ""
    kinematic_prop_visuals = ""
    if physics_mode == "hydro":
        hydro_world_plugins = """
    <plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
      <graded_buoyancy>
        <default_density>1000</default_density>
        <density_change>
          <above_depth>0</above_depth>
          <density>1</density>
        </density_change>
      </graded_buoyancy>
    </plugin>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>"""
    elif rov_variant == "github_blue":
        kinematic_prop_visuals = """
    <include>
      <name>gamantaray_rov_thruster1_prop_visual</name>
      <uri>model://gamantaray_blue_t200_prop_ccw_visual</uri>
      <pose>-3.0929 -0.0704 -0.2800 -1.571 1.571 -0.785</pose>
    </include>
    <include>
      <name>gamantaray_rov_thruster2_prop_visual</name>
      <uri>model://gamantaray_blue_t200_prop_ccw_visual</uri>
      <pose>-3.0929 0.0704 -0.2800 -1.571 1.571 -2.356</pose>
    </include>
    <include>
      <name>gamantaray_rov_thruster3_prop_visual</name>
      <uri>model://gamantaray_blue_t200_prop_cw_visual</uri>
      <pose>-3.3148 -0.0704 -0.2800 -1.571 1.571 0.785</pose>
    </include>
    <include>
      <name>gamantaray_rov_thruster4_prop_visual</name>
      <uri>model://gamantaray_blue_t200_prop_cw_visual</uri>
      <pose>-3.3148 0.0704 -0.2800 -1.571 1.571 2.356</pose>
    </include>
    <include>
      <name>gamantaray_rov_thruster5_prop_visual</name>
      <uri>model://gamantaray_blue_t200_prop_ccw_visual</uri>
      <pose>-3.2000 -0.0834 -0.2211 0 0 0</pose>
    </include>
    <include>
      <name>gamantaray_rov_thruster6_prop_visual</name>
      <uri>model://gamantaray_blue_t200_prop_cw_visual</uri>
      <pose>-3.2000 0.0834 -0.2211 0 0 0</pose>
    </include>"""
    elif rov_variant == "bluerov":
        kinematic_prop_visuals = """
    <include>
      <name>gamantaray_rov_thruster1_prop_visual</name>
      <uri>model://gamantaray_propeller_cw_visual</uri>
      <pose>-3.0889 -0.0820 -0.3395 0 0 0.7853981634</pose>
    </include>
    <include>
      <name>gamantaray_rov_thruster2_prop_visual</name>
      <uri>model://gamantaray_propeller_cw_visual</uri>
      <pose>-3.0889 0.0820 -0.3395 0 0 -0.7853981634</pose>
    </include>
    <include>
      <name>gamantaray_rov_thruster3_prop_visual</name>
      <uri>model://gamantaray_propeller_ccw_visual</uri>
      <pose>-3.3210 -0.0820 -0.3395 0 0 2.3561944902</pose>
    </include>
    <include>
      <name>gamantaray_rov_thruster4_prop_visual</name>
      <uri>model://gamantaray_propeller_ccw_visual</uri>
      <pose>-3.3210 0.0820 -0.3395 0 0 -2.3561944902</pose>
    </include>
    <include>
      <name>gamantaray_rov_thruster5_prop_visual</name>
      <uri>model://gamantaray_propeller_cw_visual</uri>
      <pose>-3.1979 -0.0906 -0.2841 0 -1.5707963268 0</pose>
    </include>
    <include>
      <name>gamantaray_rov_thruster6_prop_visual</name>
      <uri>model://gamantaray_propeller_ccw_visual</uri>
      <pose>-3.1979 0.0906 -0.2841 0 -1.5707963268 0</pose>
    </include>"""
    world_text = (
        template_path.read_text(encoding="utf-8")
        .replace("@PAYLOAD_CODE@", payload_code)
        .replace("@ROV_MODEL_URI@", rov_model_uri)
        .replace("@LEFT_GRIPPER_JAW_URI@", left_gripper_jaw_uri)
        .replace("@RIGHT_GRIPPER_JAW_URI@", right_gripper_jaw_uri)
        .replace("@PHYSICS_STEP_SIZE@", physics_step_size)
        .replace("@HYDRO_WORLD_PLUGINS@", hydro_world_plugins)
        .replace("@KINEMATIC_PROP_VISUALS@", kinematic_prop_visuals)
    )
    generated_world.write_text(world_text, encoding="utf-8")

    resource_path_parts = [
        str(Path(desc_share) / "models"),
        str(Path(gazebo_share) / "models"),
        desc_share,
        gazebo_share,
    ]
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_resource_path:
        resource_path_parts.append(existing_resource_path)

    gz_args = ["-r", "-v", "2"]
    if not gui:
        gz_args.append("-s")
    gz_args.append(str(generated_world))

    bridges = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/rov/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/rov/camera/wall/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/rov/camera/wall/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/rov/camera/bottom/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/rov/camera/bottom/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/rov/thruster1/cmd@std_msgs/msg/Float64]gz.msgs.Double",
        "/rov/thruster2/cmd@std_msgs/msg/Float64]gz.msgs.Double",
        "/rov/thruster3/cmd@std_msgs/msg/Float64]gz.msgs.Double",
        "/rov/thruster4/cmd@std_msgs/msg/Float64]gz.msgs.Double",
        "/rov/thruster5/cmd@std_msgs/msg/Float64]gz.msgs.Double",
        "/rov/thruster6/cmd@std_msgs/msg/Float64]gz.msgs.Double",
    ]
    if physics_mode == "hydro" and hydro_control_mode == "wrench":
        bridges.append("/model/gamantaray_rov/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry")

    actions = [
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=os.pathsep.join(resource_path_parts),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": " ".join(gz_args)}.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="rov_gz_bridge",
            output="screen",
            arguments=bridges,
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="rov_gamantaray_control",
            executable="cmd_vel_mux",
            name="cmd_vel_mux",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "default_source": command_source,
                }
            ],
        ),
        Node(
            package="rov_gamantaray_control",
            executable="thruster_allocator",
            name="thruster_allocator",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ]

    if physics_mode == "kinematic" or hydro_control_mode == "kinematic":
        actions.append(
            Node(
                package="rov_gamantaray_control",
                executable="kinematic_driver",
                name="kinematic_driver",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_name": "kki_rov_pool",
                        "initial_z": -0.28,
                        "bottom_limit_z": -0.72,
                        "surface_limit_z": -0.08,
                        "prop_visuals_enabled": prop_layout != "none",
                        "prop_layout": prop_layout,
                    }
                ],
            )
        )
    else:
        actions.append(
            Node(
                package="rov_gamantaray_control",
                executable="hydro_wrench_driver",
                name="hydro_wrench_driver",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_name": "kki_rov_pool",
                        "entity_name": "gamantaray_rov",
                        "entity_type": "model",
                        "horizontal_force_gain": hydro_horizontal_force_gain,
                        "vertical_force_gain": hydro_vertical_force_gain,
                        "yaw_torque_gain": hydro_yaw_torque_gain,
                    }
                ],
            )
        )

    actions.extend(
        [
            Node(
                package="rov_gamantaray_control",
                executable="gripper_manager",
                name="gripper_manager",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_name": "kki_rov_pool",
                        "payload_code": payload_code,
                        "hanging_constraint_enabled": hanging_constraint_enabled,
                        "hanging_damping": hanging_damping,
                        "hanging_release_velocity_gain": hanging_release_velocity_gain,
                        "hanging_max_angle_rad": hanging_max_angle_rad,
                    }
                ],
            ),
            Node(
                package="rov_gamantaray_control",
                executable="water_effects_driver",
                name="water_effects_driver",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_name": "kki_rov_pool",
                    }
                ],
            ),
            Node(
                package="rov_gamantaray_vision",
                executable="qr_detector",
                name="qr_detector",
                output="screen",
                condition=IfCondition(use_vision),
                parameters=[
                    {
                        "use_sim_time": True,
                        "image_topic": qr_image_topic,
                    }
                ],
            ),
            Node(
                package="rov_gamantaray_control",
                executable="mission_supervisor",
                name="mission_supervisor",
                output="screen",
                condition=IfCondition(mission_autonomy),
                parameters=[
                    {
                        "use_sim_time": True,
                        "default_payload_code": payload_code,
                        "mission_profile": mission_profile,
                        "auto_start_on_attached": auto_start_on_attached,
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
                        "use_sim_time": True,
                        "device_path": joystick_device,
                        "axis_surge": joystick_axis_surge,
                        "axis_sway": joystick_axis_sway,
                        "axis_heave": joystick_axis_heave,
                        "axis_yaw": joystick_axis_yaw,
                        "invert_surge": joystick_invert_surge,
                        "invert_sway": joystick_invert_sway,
                        "invert_heave": joystick_invert_heave,
                        "invert_yaw": joystick_invert_yaw,
                        "deadzone": joystick_deadzone,
                        "linear_scale": joystick_linear_scale,
                        "vertical_scale": joystick_vertical_scale,
                        "yaw_scale": joystick_yaw_scale,
                        "enable_button": joystick_enable_button,
                    }
                ],
            ),
        ]
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("payload_code", default_value="A"),
            DeclareLaunchArgument("use_vision", default_value="false"),
            DeclareLaunchArgument("qr_image_topic", default_value="/rov/camera/wall/image"),
            DeclareLaunchArgument("mission_autonomy", default_value="false"),
            DeclareLaunchArgument("mission_profile", default_value="full"),
            DeclareLaunchArgument("auto_start_on_attached", default_value="false"),
            DeclareLaunchArgument("command_source", default_value="auto_if_mission"),
            DeclareLaunchArgument("joystick", default_value="false"),
            DeclareLaunchArgument("joystick_device", default_value="/dev/input/js0"),
            DeclareLaunchArgument("joystick_axis_surge", default_value="1"),
            DeclareLaunchArgument("joystick_axis_sway", default_value="0"),
            DeclareLaunchArgument("joystick_axis_heave", default_value="3"),
            DeclareLaunchArgument("joystick_axis_yaw", default_value="2"),
            DeclareLaunchArgument("joystick_invert_surge", default_value="true"),
            DeclareLaunchArgument("joystick_invert_sway", default_value="true"),
            DeclareLaunchArgument("joystick_invert_heave", default_value="true"),
            DeclareLaunchArgument("joystick_invert_yaw", default_value="false"),
            DeclareLaunchArgument("joystick_deadzone", default_value="0.08"),
            DeclareLaunchArgument("joystick_linear_scale", default_value="0.75"),
            DeclareLaunchArgument("joystick_vertical_scale", default_value="0.55"),
            DeclareLaunchArgument("joystick_yaw_scale", default_value="0.65"),
            DeclareLaunchArgument("joystick_enable_button", default_value="-1"),
            DeclareLaunchArgument("hydro_horizontal_force_gain", default_value="2.00"),
            DeclareLaunchArgument("hydro_vertical_force_gain", default_value="0.80"),
            DeclareLaunchArgument("hydro_yaw_torque_gain", default_value="0.20"),
            DeclareLaunchArgument("hanging_constraint_enabled", default_value="true"),
            DeclareLaunchArgument("hanging_damping", default_value="2.40"),
            DeclareLaunchArgument("hanging_release_velocity_gain", default_value="0.45"),
            DeclareLaunchArgument("hanging_max_angle_rad", default_value="0.45"),
            DeclareLaunchArgument("physics_mode", default_value="kinematic"),
            DeclareLaunchArgument("hydro_control_mode", default_value="kinematic"),
            DeclareLaunchArgument("rov_variant", default_value="github_blue"),
            OpaqueFunction(function=launch_setup),
        ]
    )
