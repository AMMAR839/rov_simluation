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


def make_tether_models(segment_count: int) -> str:
    parts = [
        """
    <model name="rov_tether_anchor">
      <static>true</static>
      <pose>-4.55 -4.55 0.08 0 0 0</pose>
      <link name="link">
        <visual name="deck_plate"><pose>0 0 0.025 0 0 0</pose><geometry><box><size>0.34 0.24 0.05</size></box></geometry><material><ambient>0.78 0.36 0.06 1</ambient><diffuse>0.95 0.48 0.08 1</diffuse></material></visual>
        <visual name="spool"><pose>0 0 0.115 1.5708 0 0</pose><geometry><cylinder><radius>0.075</radius><length>0.20</length></cylinder></geometry><material><ambient>0.08 0.09 0.10 1</ambient><diffuse>0.13 0.14 0.15 1</diffuse></material></visual>
        <visual name="orange_winding"><pose>0 0 0.115 1.5708 0 0</pose><geometry><cylinder><radius>0.083</radius><length>0.12</length></cylinder></geometry><material><ambient>0.95 0.38 0.05 1</ambient><diffuse>1.00 0.48 0.08 1</diffuse></material></visual>
        <visual name="mast"><pose>0 0 0.28 0 0 0</pose><geometry><cylinder><radius>0.012</radius><length>0.38</length></cylinder></geometry><material><ambient>0.80 0.83 0.80 1</ambient><diffuse>0.90 0.92 0.88 1</diffuse></material></visual>
      </link>
    </model>"""
    ]
    for index in range(1, segment_count + 1):
        parts.append(
            f"""
    <model name="rov_tether_segment_{index:02d}">
      <static>true</static>
      <pose>0 0 6 0 0 0</pose>
      <link name="link">
        <visual name="tether_bead">
          <geometry><sphere><radius>0.015</radius></sphere></geometry>
          <material><ambient>0.95 0.36 0.05 1</ambient><diffuse>1.0 0.48 0.08 1</diffuse><specular>0.25 0.16 0.08 1</specular></material>
        </visual>
      </link>
    </model>"""
        )
    return "\n".join(parts)


def launch_setup(context, *args, **kwargs):
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    kki_gui = LaunchConfiguration("kki_gui")
    kki_gui_window = LaunchConfiguration("kki_gui_window")
    team_name = LaunchConfiguration("team_name")
    university_name = LaunchConfiguration("university_name")
    tether = LaunchConfiguration("tether")
    tether_enabled = tether.perform(context).lower() == "true"
    tether_segment_count = LaunchConfiguration("tether_segment_count")
    tether_segment_count_value = int(tether_segment_count.perform(context))
    tether_max_length_m = LaunchConfiguration("tether_max_length_m")
    use_vision = LaunchConfiguration("use_vision")
    qr_image_topic = LaunchConfiguration("qr_image_topic")
    mission_autonomy = LaunchConfiguration("mission_autonomy")
    mission_autonomy_enabled = mission_autonomy.perform(context).lower() == "true"
    mission_profile = LaunchConfiguration("mission_profile")
    auto_start_on_attached = LaunchConfiguration("auto_start_on_attached")
    require_qr_for_target = LaunchConfiguration("require_qr_for_target")
    use_default_payload_after_scan_timeout = LaunchConfiguration(
        "use_default_payload_after_scan_timeout"
    )
    release_requires_attached = LaunchConfiguration("release_requires_attached")
    release_surface_on_timeout = LaunchConfiguration("release_surface_on_timeout")
    jaw_pivot_x_m = LaunchConfiguration("jaw_pivot_x_m")
    jaw_pivot_y_m = LaunchConfiguration("jaw_pivot_y_m")
    jaw_open_angle_rad = LaunchConfiguration("jaw_open_angle_rad")
    jaw_closed_angle_rad = LaunchConfiguration("jaw_closed_angle_rad")
    capture_forward_offset_m = LaunchConfiguration("capture_forward_offset_m")
    capture_vertical_offset_m = LaunchConfiguration("capture_vertical_offset_m")
    closed_gap_m = LaunchConfiguration("closed_gap_m")
    open_gap_m = LaunchConfiguration("open_gap_m")
    mouth_clearance_m = LaunchConfiguration("mouth_clearance_m")
    payload_funnel_response_s = LaunchConfiguration("payload_funnel_response_s")
    capture_alignment_hold_s = LaunchConfiguration("capture_alignment_hold_s")
    capture_forward_tolerance_m = LaunchConfiguration("capture_forward_tolerance_m")
    capture_lateral_tolerance_m = LaunchConfiguration("capture_lateral_tolerance_m")
    held_payload_response_s = LaunchConfiguration("held_payload_response_s")
    held_payload_max_sway_m = LaunchConfiguration("held_payload_max_sway_m")
    payload_contact_model = LaunchConfiguration("payload_contact_model")
    payload_contact_resolution_gain = LaunchConfiguration("payload_contact_resolution_gain")
    payload_contact_max_step_m = LaunchConfiguration("payload_contact_max_step_m")
    grip_min_bilateral_contact_s = LaunchConfiguration("grip_min_bilateral_contact_s")
    grip_clamp_margin_m = LaunchConfiguration("grip_clamp_margin_m")
    jaw_visual_stop_clearance_m = LaunchConfiguration("jaw_visual_stop_clearance_m")
    grip_required_depth_error_m = LaunchConfiguration("grip_required_depth_error_m")
    grip_required_height_error_m = LaunchConfiguration("grip_required_height_error_m")
    gripper_frame_backstop_x_m = LaunchConfiguration("gripper_frame_backstop_x_m")
    payload_drop_buoyancy_ratio = LaunchConfiguration("payload_drop_buoyancy_ratio")
    payload_drop_linear_drag = LaunchConfiguration("payload_drop_linear_drag")
    payload_drop_terminal_speed_mps = LaunchConfiguration("payload_drop_terminal_speed_mps")
    hook_snap_rov_tolerance_m = LaunchConfiguration("hook_snap_rov_tolerance_m")
    hook_snap_hole_tolerance_m = LaunchConfiguration("hook_snap_hole_tolerance_m")
    hook_snap_axial_tolerance_m = LaunchConfiguration("hook_snap_axial_tolerance_m")
    hook_snap_yaw_tolerance_rad = LaunchConfiguration("hook_snap_yaw_tolerance_rad")
    hook_peg_pass_window_m = LaunchConfiguration("hook_peg_pass_window_m")
    hook_latch_release_tolerance_m = LaunchConfiguration("hook_latch_release_tolerance_m")
    hook_latch_axial_release_tolerance_m = LaunchConfiguration(
        "hook_latch_axial_release_tolerance_m"
    )
    hook_latch_memory_s = LaunchConfiguration("hook_latch_memory_s")
    hook_entry_tip_min_t = LaunchConfiguration("hook_entry_tip_min_t")
    hook_entry_memory_s = LaunchConfiguration("hook_entry_memory_s")
    hook_collision_guard_enabled = LaunchConfiguration("hook_collision_guard_enabled")
    hook_collision_body_radius_m = LaunchConfiguration("hook_collision_body_radius_m")
    hook_collision_margin_m = LaunchConfiguration("hook_collision_margin_m")
    hook_collision_body_half_height_m = LaunchConfiguration(
        "hook_collision_body_half_height_m"
    )
    hook_collision_velocity_damping = LaunchConfiguration(
        "hook_collision_velocity_damping"
    )
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
    pwm_neutral_us = LaunchConfiguration("pwm_neutral_us")
    pwm_min_us = LaunchConfiguration("pwm_min_us")
    pwm_max_us = LaunchConfiguration("pwm_max_us")
    pwm_deadband_us = LaunchConfiguration("pwm_deadband_us")
    pwm_response_s = LaunchConfiguration("pwm_response_s")
    hanging_constraint_enabled = LaunchConfiguration("hanging_constraint_enabled")
    hanging_damping = LaunchConfiguration("hanging_damping")
    hanging_release_velocity_gain = LaunchConfiguration("hanging_release_velocity_gain")
    hanging_max_angle_rad = LaunchConfiguration("hanging_max_angle_rad")
    physics_mode = LaunchConfiguration("physics_mode").perform(context).lower()
    hydro_control_mode = LaunchConfiguration("hydro_control_mode").perform(context).lower()
    rov_variant = LaunchConfiguration("rov_variant").perform(context).lower()
    gripper_geometry = LaunchConfiguration("gripper_geometry").perform(context).lower()
    gripper_actuation_arg = LaunchConfiguration("gripper_actuation_mode").perform(context).lower()
    payload_contact_response_arg = LaunchConfiguration(
        "payload_contact_response_enabled"
    ).perform(context).lower()
    payload_code = LaunchConfiguration("payload_code").perform(context).upper()
    if payload_code not in VALID_PAYLOADS:
        raise RuntimeError("payload_code must be one of A, B, C, or D")
    if physics_mode not in {"kinematic", "hydro"}:
        raise RuntimeError("physics_mode must be kinematic or hydro")
    if hydro_control_mode not in {"kinematic", "wrench"}:
        raise RuntimeError("hydro_control_mode must be kinematic or wrench")
    valid_rov_variants = {
        "github_blue",
        "github_blue_joint",
        "github_blue_joint_experimental",
        "beaumont",
        "bluerov",
    }
    if rov_variant not in valid_rov_variants:
        raise RuntimeError(
            "rov_variant must be github_blue, github_blue_joint, "
            "github_blue_joint_experimental, beaumont, or bluerov"
        )
    if gripper_geometry not in {"auto", "manual"}:
        raise RuntimeError("gripper_geometry must be auto or manual")
    if gripper_actuation_arg not in {"auto", "kinematic", "joint"}:
        raise RuntimeError("gripper_actuation_mode must be auto, kinematic, or joint")
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
    if gripper_actuation_arg == "auto":
        gripper_actuation_mode = (
            "joint"
            if physics_mode == "kinematic"
            and rov_variant == "github_blue_joint_experimental"
            else "kinematic"
        )
    else:
        gripper_actuation_mode = gripper_actuation_arg
    if gripper_actuation_mode == "joint" and rov_variant != "github_blue_joint_experimental":
        raise RuntimeError(
            "gripper_actuation_mode:=joint requires "
            "rov_variant:=github_blue_joint_experimental"
        )
    if gripper_actuation_mode == "joint" and physics_mode != "kinematic":
        raise RuntimeError(
            "gripper_actuation_mode:=joint is currently supported in "
            "physics_mode:=kinematic"
        )

    if physics_mode == "hydro":
        rov_model_uri = "gamantaray_rov_hydro"
        left_gripper_jaw_uri = "gamantaray_gripper_ricketts_left_jaw_visual"
        right_gripper_jaw_uri = "gamantaray_gripper_ricketts_right_jaw_visual"
    elif rov_variant == "github_blue_joint_experimental":
        rov_model_uri = "gamantaray_rov_github_blue_joint_gripper"
        left_gripper_jaw_uri = ""
        right_gripper_jaw_uri = ""
        prop_layout = "github_blue"
    elif rov_variant in {"github_blue", "github_blue_joint"}:
        rov_model_uri = "gamantaray_rov_github_blue_gripper"
        left_gripper_jaw_uri = "gamantaray_gripper_ricketts_left_jaw_visual"
        right_gripper_jaw_uri = "gamantaray_gripper_ricketts_right_jaw_visual"
        prop_layout = "github_blue"
    elif rov_variant == "beaumont":
        rov_model_uri = "gamantaray_rov_beaumont_gripper"
        left_gripper_jaw_uri = "gamantaray_gripper_beaumont_left_jaw_visual"
        right_gripper_jaw_uri = "gamantaray_gripper_beaumont_right_jaw_visual"
    else:
        rov_model_uri = "gamantaray_rov"
        left_gripper_jaw_uri = "gamantaray_gripper_left_jaw_visual"
        right_gripper_jaw_uri = "gamantaray_gripper_right_jaw_visual"
        prop_layout = "legacy_bluerov"

    if gripper_geometry == "auto":
        if rov_variant == "beaumont":
            gripper_values = {
                "jaw_pivot_x_m": "0.260",
                "jaw_pivot_y_m": "0.064",
                "jaw_open_angle_rad": "0.50",
                "jaw_closed_angle_rad": "0.035",
                "capture_forward_offset_m": "0.365",
                "capture_vertical_offset_m": "-0.136",
                "capture_forward_tolerance_m": "0.095",
                "capture_lateral_tolerance_m": "0.052",
                "closed_gap_m": "0.050",
                "open_gap_m": "0.165",
                "mouth_clearance_m": "0.006",
            }
        else:
            gripper_values = {
                "jaw_pivot_x_m": "0.244",
                "jaw_pivot_y_m": "0.050",
                "jaw_open_angle_rad": "0.38",
                "jaw_closed_angle_rad": "0.035",
                "capture_forward_offset_m": "0.335",
                "capture_vertical_offset_m": "-0.112",
                "capture_forward_tolerance_m": "0.040",
                "capture_lateral_tolerance_m": "0.035",
                "closed_gap_m": "0.050",
                "open_gap_m": "0.125",
                "mouth_clearance_m": "0.006",
            }
    else:
        gripper_values = {
            "jaw_pivot_x_m": jaw_pivot_x_m,
            "jaw_pivot_y_m": jaw_pivot_y_m,
            "jaw_open_angle_rad": jaw_open_angle_rad,
            "jaw_closed_angle_rad": jaw_closed_angle_rad,
            "capture_forward_offset_m": capture_forward_offset_m,
            "capture_vertical_offset_m": capture_vertical_offset_m,
            "capture_forward_tolerance_m": capture_forward_tolerance_m,
            "capture_lateral_tolerance_m": capture_lateral_tolerance_m,
            "closed_gap_m": closed_gap_m,
            "open_gap_m": open_gap_m,
            "mouth_clearance_m": mouth_clearance_m,
        }
    gripper_values = {
        key: (value.perform(context) if hasattr(value, "perform") else str(value))
        for key, value in gripper_values.items()
    }
    gripper_values = {key: float(value) for key, value in gripper_values.items()}
    if payload_contact_response_arg == "auto":
        payload_contact_response_enabled = (
            False if gripper_actuation_mode == "joint" else True
        )
    else:
        payload_contact_response_enabled = payload_contact_response_arg in {
            "1",
            "true",
            "yes",
            "on",
        }

    if gripper_actuation_mode == "joint":
        gripper_jaw_includes = ""
    else:
        jaw_x = -3.2 + gripper_values["jaw_pivot_x_m"]
        jaw_y = gripper_values["jaw_pivot_y_m"]
        jaw_z = -0.28 + gripper_values["capture_vertical_offset_m"]
        jaw_open = gripper_values["jaw_open_angle_rad"]
        gripper_jaw_includes = f"""
    <include>
      <name>gamantaray_rov_left_gripper_jaw</name>
      <uri>model://{left_gripper_jaw_uri}</uri>
      <pose>{jaw_x:.3f} {jaw_y:.3f} {jaw_z:.3f} 0 0 {jaw_open:.3f}</pose>
    </include>
    <include>
      <name>gamantaray_rov_right_gripper_jaw</name>
      <uri>model://{right_gripper_jaw_uri}</uri>
      <pose>{jaw_x:.3f} {-jaw_y:.3f} {jaw_z:.3f} 0 0 {-jaw_open:.3f}</pose>
    </include>"""
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
    elif rov_variant in {
        "github_blue",
        "github_blue_joint",
        "github_blue_joint_experimental",
    }:
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
        .replace("@GRIPPER_JAW_INCLUDES@", gripper_jaw_includes)
        .replace("@PHYSICS_STEP_SIZE@", physics_step_size)
        .replace("@HYDRO_WORLD_PLUGINS@", hydro_world_plugins)
        .replace("@KINEMATIC_PROP_VISUALS@", kinematic_prop_visuals)
        .replace(
            "@TETHER_MODELS@",
            make_tether_models(tether_segment_count_value) if tether_enabled else "",
        )
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
        "/rov/thruster1/pwm@std_msgs/msg/Float64]gz.msgs.Double",
        "/rov/thruster2/pwm@std_msgs/msg/Float64]gz.msgs.Double",
        "/rov/thruster3/pwm@std_msgs/msg/Float64]gz.msgs.Double",
        "/rov/thruster4/pwm@std_msgs/msg/Float64]gz.msgs.Double",
        "/rov/thruster5/pwm@std_msgs/msg/Float64]gz.msgs.Double",
        "/rov/thruster6/pwm@std_msgs/msg/Float64]gz.msgs.Double",
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
            parameters=[
                {
                    "use_sim_time": True,
                    "pwm_neutral_us": pwm_neutral_us,
                    "pwm_min_us": pwm_min_us,
                    "pwm_max_us": pwm_max_us,
                    "pwm_deadband_us": pwm_deadband_us,
                    "pwm_response_s": pwm_response_s,
                }
            ],
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
                        "pwm_neutral_us": pwm_neutral_us,
                        "pwm_min_us": pwm_min_us,
                        "pwm_max_us": pwm_max_us,
                        "pwm_deadband_us": pwm_deadband_us,
                        "hook_collision_guard_enabled": hook_collision_guard_enabled,
                        "hook_collision_body_radius_m": hook_collision_body_radius_m,
                        "hook_collision_margin_m": hook_collision_margin_m,
                        "hook_collision_body_half_height_m": hook_collision_body_half_height_m,
                        "hook_collision_velocity_damping": hook_collision_velocity_damping,
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
                        "pwm_neutral_us": pwm_neutral_us,
                        "pwm_min_us": pwm_min_us,
                        "pwm_max_us": pwm_max_us,
                        "pwm_deadband_us": pwm_deadband_us,
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
                        "jaw_pivot_x_m": gripper_values["jaw_pivot_x_m"],
                        "jaw_pivot_y_m": gripper_values["jaw_pivot_y_m"],
                        "jaw_open_angle_rad": gripper_values["jaw_open_angle_rad"],
                        "jaw_closed_angle_rad": gripper_values["jaw_closed_angle_rad"],
                        "capture_forward_offset_m": gripper_values["capture_forward_offset_m"],
                        "capture_vertical_offset_m": gripper_values["capture_vertical_offset_m"],
                        "mouth_clearance_m": gripper_values["mouth_clearance_m"],
                        "closed_gap_m": gripper_values["closed_gap_m"],
                        "open_gap_m": gripper_values["open_gap_m"],
                        "payload_funnel_response_s": payload_funnel_response_s,
                        "payload_contact_model": payload_contact_model,
                        "payload_contact_response_enabled": payload_contact_response_enabled,
                        "payload_contact_resolution_gain": payload_contact_resolution_gain,
                        "payload_contact_max_step_m": payload_contact_max_step_m,
                        "grip_attach_requires_bilateral_contact": True,
                        "grip_min_bilateral_contact_s": grip_min_bilateral_contact_s,
                        "grip_clamp_margin_m": grip_clamp_margin_m,
                        "jaw_visual_stop_clearance_m": jaw_visual_stop_clearance_m,
                        "grip_required_depth_error_m": grip_required_depth_error_m,
                        "grip_required_height_error_m": grip_required_height_error_m,
                        "gripper_frame_backstop_x_m": gripper_frame_backstop_x_m,
                        "gripper_actuation_mode": gripper_actuation_mode,
                        "capture_alignment_hold_s": capture_alignment_hold_s,
                        "capture_forward_tolerance_m": gripper_values["capture_forward_tolerance_m"],
                        "capture_lateral_tolerance_m": gripper_values["capture_lateral_tolerance_m"],
                        "held_payload_response_s": held_payload_response_s,
                        "held_payload_max_sway_m": held_payload_max_sway_m,
                        "payload_drop_buoyancy_ratio": payload_drop_buoyancy_ratio,
                        "payload_drop_linear_drag": payload_drop_linear_drag,
                        "payload_drop_terminal_speed_mps": payload_drop_terminal_speed_mps,
                        "hook_snap_rov_tolerance_m": hook_snap_rov_tolerance_m,
                        "hook_snap_hole_tolerance_m": hook_snap_hole_tolerance_m,
                        "hook_snap_axial_tolerance_m": hook_snap_axial_tolerance_m,
                        "hook_snap_yaw_tolerance_rad": hook_snap_yaw_tolerance_rad,
                        "hook_peg_pass_window_m": hook_peg_pass_window_m,
                        "hook_latch_release_tolerance_m": hook_latch_release_tolerance_m,
                        "hook_latch_axial_release_tolerance_m": hook_latch_axial_release_tolerance_m,
                        "hook_latch_memory_s": hook_latch_memory_s,
                        "hook_entry_tip_min_t": hook_entry_tip_min_t,
                        "hook_entry_memory_s": hook_entry_memory_s,
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
                package="rov_gamantaray_control",
                executable="tether_driver",
                name="tether_driver",
                output="screen",
                condition=IfCondition(tether),
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_name": "kki_rov_pool",
                        "segment_count": tether_segment_count,
                        "max_length_m": tether_max_length_m,
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
                executable="kki_dashboard",
                name="kki_dashboard",
                output="screen",
                condition=IfCondition(kki_gui),
                parameters=[
                    {
                        "use_sim_time": True,
                        "team_name": team_name,
                        "university_name": university_name,
                        "window_enabled": kki_gui_window,
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
                        "require_qr_for_target": require_qr_for_target,
                        "use_default_payload_after_scan_timeout": use_default_payload_after_scan_timeout,
                        "release_requires_attached": release_requires_attached,
                        "release_surface_on_timeout": release_surface_on_timeout,
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
            DeclareLaunchArgument("kki_gui", default_value="false"),
            DeclareLaunchArgument("kki_gui_window", default_value="true"),
            DeclareLaunchArgument("team_name", default_value="Gamantara"),
            DeclareLaunchArgument("university_name", default_value="Universitas Gadjah Mada"),
            DeclareLaunchArgument("tether", default_value="true"),
            DeclareLaunchArgument("tether_segment_count", default_value="28"),
            DeclareLaunchArgument("tether_max_length_m", default_value="12.0"),
            DeclareLaunchArgument("payload_code", default_value="A"),
            DeclareLaunchArgument("use_vision", default_value="false"),
            DeclareLaunchArgument("qr_image_topic", default_value="/rov/camera/wall/image"),
            DeclareLaunchArgument("mission_autonomy", default_value="false"),
            DeclareLaunchArgument("mission_profile", default_value="full"),
            DeclareLaunchArgument("auto_start_on_attached", default_value="false"),
            DeclareLaunchArgument("require_qr_for_target", default_value="true"),
            DeclareLaunchArgument("use_default_payload_after_scan_timeout", default_value="false"),
            DeclareLaunchArgument("release_requires_attached", default_value="true"),
            DeclareLaunchArgument("release_surface_on_timeout", default_value="false"),
            DeclareLaunchArgument("gripper_geometry", default_value="auto"),
            DeclareLaunchArgument("gripper_actuation_mode", default_value="auto"),
            DeclareLaunchArgument("payload_contact_response_enabled", default_value="auto"),
            DeclareLaunchArgument("payload_contact_model", default_value="strict"),
            DeclareLaunchArgument("payload_contact_resolution_gain", default_value="1.00"),
            DeclareLaunchArgument("payload_contact_max_step_m", default_value="0.016"),
            DeclareLaunchArgument("grip_min_bilateral_contact_s", default_value="0.20"),
            DeclareLaunchArgument("grip_clamp_margin_m", default_value="0.006"),
            DeclareLaunchArgument("jaw_visual_stop_clearance_m", default_value="0.014"),
            DeclareLaunchArgument("grip_required_depth_error_m", default_value="0.040"),
            DeclareLaunchArgument("grip_required_height_error_m", default_value="0.050"),
            DeclareLaunchArgument("gripper_frame_backstop_x_m", default_value="0.330"),
            DeclareLaunchArgument("jaw_pivot_x_m", default_value="0.244"),
            DeclareLaunchArgument("jaw_pivot_y_m", default_value="0.050"),
            DeclareLaunchArgument("jaw_open_angle_rad", default_value="0.38"),
            DeclareLaunchArgument("jaw_closed_angle_rad", default_value="0.035"),
            DeclareLaunchArgument("capture_forward_offset_m", default_value="0.335"),
            DeclareLaunchArgument("capture_vertical_offset_m", default_value="-0.112"),
            DeclareLaunchArgument("closed_gap_m", default_value="0.050"),
            DeclareLaunchArgument("open_gap_m", default_value="0.125"),
            DeclareLaunchArgument("mouth_clearance_m", default_value="0.006"),
            DeclareLaunchArgument("payload_funnel_response_s", default_value="0.12"),
            DeclareLaunchArgument("capture_alignment_hold_s", default_value="0.15"),
            DeclareLaunchArgument("capture_forward_tolerance_m", default_value="0.040"),
            DeclareLaunchArgument("capture_lateral_tolerance_m", default_value="0.035"),
            DeclareLaunchArgument("held_payload_response_s", default_value="0.06"),
            DeclareLaunchArgument("held_payload_max_sway_m", default_value="0.020"),
            DeclareLaunchArgument("payload_drop_buoyancy_ratio", default_value="0.72"),
            DeclareLaunchArgument("payload_drop_linear_drag", default_value="4.2"),
            DeclareLaunchArgument("payload_drop_terminal_speed_mps", default_value="0.22"),
            DeclareLaunchArgument("hook_snap_rov_tolerance_m", default_value="0.30"),
            DeclareLaunchArgument("hook_snap_hole_tolerance_m", default_value="0.024"),
            DeclareLaunchArgument("hook_snap_axial_tolerance_m", default_value="0.026"),
            DeclareLaunchArgument("hook_snap_yaw_tolerance_rad", default_value="0.35"),
            DeclareLaunchArgument("hook_peg_pass_window_m", default_value="0.028"),
            DeclareLaunchArgument("hook_latch_release_tolerance_m", default_value="0.036"),
            DeclareLaunchArgument(
                "hook_latch_axial_release_tolerance_m", default_value="0.075"
            ),
            DeclareLaunchArgument("hook_latch_memory_s", default_value="3.00"),
            DeclareLaunchArgument("hook_entry_tip_min_t", default_value="0.78"),
            DeclareLaunchArgument("hook_entry_memory_s", default_value="4.00"),
            DeclareLaunchArgument("hook_collision_guard_enabled", default_value="true"),
            DeclareLaunchArgument("hook_collision_body_radius_m", default_value="0.17"),
            DeclareLaunchArgument("hook_collision_margin_m", default_value="0.035"),
            DeclareLaunchArgument("hook_collision_body_half_height_m", default_value="0.18"),
            DeclareLaunchArgument("hook_collision_velocity_damping", default_value="0.20"),
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
            DeclareLaunchArgument("pwm_neutral_us", default_value="1500.0"),
            DeclareLaunchArgument("pwm_min_us", default_value="1100.0"),
            DeclareLaunchArgument("pwm_max_us", default_value="1900.0"),
            DeclareLaunchArgument("pwm_deadband_us", default_value="25.0"),
            DeclareLaunchArgument("pwm_response_s", default_value="0.04"),
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
