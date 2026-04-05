import os
import re
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    def _has_nvidia_ml():
        candidates = [
            "/usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1",
            "/usr/lib64/libnvidia-ml.so.1",
            "/usr/lib/libnvidia-ml.so.1",
        ]
        return any(os.path.exists(path) for path in candidates)

    def _read_text(path):
        with open(path, "r", encoding="utf-8") as file_obj:
            return file_obj.read()

    def _write_temp_file(content, suffix, prefix):
        with tempfile.NamedTemporaryFile(
            "w",
            delete=False,
            encoding="utf-8",
            suffix=suffix,
            prefix=prefix,
        ) as temp_file:
            temp_file.write(content)
            return temp_file.name

    def _load_sim_config(config_path):
        default_config = {
            "lidar_mode": "auto",
            "rgl_plugin_root": "",
            "livox_mid360": {
                "horizontal_samples": 1875,
                "horizontal_min_angle": 0.0,
                "horizontal_max_angle": 6.2831852,
                "vertical_samples": 32,
                "vertical_min_angle": -0.12217304764,
                "vertical_max_angle": 0.90757121104,
                "range_min": 0.1,
                "range_max": 40.0,
                "update_rate": 10,
                "noise_type": "gaussian",
                "noise_mean": 0.0,
                "noise_stddev": 0.01,
            },
        }
        if os.path.exists(config_path):
            with open(config_path, "r", encoding="utf-8") as file_obj:
                user_config = yaml.safe_load(file_obj) or {}
            for key, default_value in default_config.items():
                if key not in user_config:
                    continue
                if isinstance(default_value, dict):
                    default_value.update(user_config[key])
                else:
                    default_config[key] = user_config[key]
        return default_config

    def _determine_use_rgl(lidar_mode, has_nvidia):
        if lidar_mode == "rgl":
            if not has_nvidia:
                print("[WARN] lidar_mode设置为'rgl'但未检测到NVIDIA显卡，强制切换为gpu_lidar")
                return False
            return True
        if lidar_mode == "gpu_lidar":
            return False
        return has_nvidia

    def _write_fallback_world(src_path):
        content = _read_text(src_path)
        plugin_start = content.find(
            "<plugin name='rgl::RGLServerPluginManager' filename='RGLServerPluginManager'>"
        )
        if plugin_start != -1:
            plugin_end = content.find("</plugin>", plugin_start)
            if plugin_end != -1:
                content = (
                    content[:plugin_start]
                    + content[plugin_end + len("</plugin>") :]
                )
        return _write_temp_file(content, ".world", "rm_sim_26_world_no_rgl_")

    def _build_gpu_lidar_sensor(lidar_config):
        return (
            '<sensor name="livox_lidar" type="gpu_lidar">\n'
            "  <pose>0 0 0 0 0 0</pose>\n"
            "  <always_on>true</always_on>\n"
            "  <visualize>true</visualize>\n"
            f'  <update_rate>{lidar_config["update_rate"]}</update_rate>\n'
            "  <topic>gz_lidar_points</topic>\n"
            "  <gz_frame_id>livox_lidar</gz_frame_id>\n"
            "  <ray>\n"
            "    <scan>\n"
            "      <horizontal>\n"
            f'        <samples>{lidar_config["horizontal_samples"]}</samples>\n'
            "        <resolution>1.0</resolution>\n"
            f'        <min_angle>{lidar_config["horizontal_min_angle"]}</min_angle>\n'
            f'        <max_angle>{lidar_config["horizontal_max_angle"]}</max_angle>\n'
            "      </horizontal>\n"
            "      <vertical>\n"
            f'        <samples>{lidar_config["vertical_samples"]}</samples>\n'
            f'        <min_angle>{lidar_config["vertical_min_angle"]}</min_angle>\n'
            f'        <max_angle>{lidar_config["vertical_max_angle"]}</max_angle>\n'
            "      </vertical>\n"
            "    </scan>\n"
            "    <range>\n"
            f'      <min>{lidar_config["range_min"]}</min>\n'
            f'      <max>{lidar_config["range_max"]}</max>\n'
            "    </range>\n"
            "    <noise>\n"
            f'      <type>{lidar_config["noise_type"]}</type>\n'
            f'      <mean>{lidar_config["noise_mean"]}</mean>\n'
            f'      <stddev>{lidar_config["noise_stddev"]}</stddev>\n'
            "    </noise>\n"
            "  </ray>\n"
            "</sensor>\n"
        )

    def _write_runtime_model(src_path, lidar_config, use_rgl, pattern_path):
        content = _read_text(src_path)
        if use_rgl:
            content, replacements = re.subn(
                r"<pattern_preset_path>.*?</pattern_preset_path>",
                f"<pattern_preset_path>{pattern_path}</pattern_preset_path>",
                content,
                count=1,
                flags=re.DOTALL,
            )
            if replacements == 0:
                print("[WARN] 未找到pattern_preset_path标签，将继续使用原始模型文件")
            return _write_temp_file(content, ".sdf", "rm_sim_26_model_rgl_")

        sensor_start = content.find('<sensor name="RGLLidar" type="custom">')
        if sensor_start != -1:
            sensor_end = content.find("</sensor>", sensor_start)
            if sensor_end != -1:
                content = (
                    content[:sensor_start]
                    + _build_gpu_lidar_sensor(lidar_config)
                    + content[sensor_end + len("</sensor>") :]
                )
        return _write_temp_file(content, ".sdf", "rm_sim_26_model_gpu_lidar_")

    def _join_env_path(variable_name, *extra_paths):
        paths = [os.environ.get(variable_name, "")]
        paths.extend(extra_paths)
        return ":".join(path for path in paths if path)

    package_name = "rm_sim_26"
    pkg_share_path = get_package_share_directory(package_name)
    world_file = os.path.join(pkg_share_path, "worlds", "rmuc_2025_world.world")
    robot_urdf_path = os.path.join(
        pkg_share_path, "models", "mecanum_car", "robot.urdf"
    )
    robot_model_path = os.path.join(
        pkg_share_path, "models", "mecanum_car", "model.sdf"
    )
    pattern_file_path = os.path.join(
        pkg_share_path, "plugin", "LivoxMid360.mat3x4f"
    )
    with open(robot_urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    config_path = os.path.join(pkg_share_path, "config", "sim_config.yaml")
    sim_config = _load_sim_config(config_path)

    has_nvidia = _has_nvidia_ml()
    use_rgl = _determine_use_rgl(sim_config["lidar_mode"], has_nvidia)

    source_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    if not os.path.isdir(os.path.join(source_root, "external", "RGLGazeboPlugin")):
        source_root = ""

    rgl_root_from_config = sim_config.get("rgl_plugin_root", "")
    rgl_root = ""
    if use_rgl:
        rgl_root = os.environ.get(
            "RGL_GZ_PLUGIN_ROOT", rgl_root_from_config or source_root
        )

    rgl_system_plugin = (
        os.path.join(
            rgl_root, "external", "RGLGazeboPlugin", "install", "RGLServerPlugin"
        )
        if rgl_root
        else ""
    )
    rgl_gui_plugin = (
        os.path.join(
            rgl_root, "external", "RGLGazeboPlugin", "install", "RGLVisualize"
        )
        if rgl_root
        else ""
    )
    patterns_dir_fallback = (
        os.path.join(rgl_root, "external", "RGLGazeboPlugin", "lidar_patterns")
        if rgl_root
        else ""
    )

    if use_rgl and not os.path.isdir(rgl_system_plugin):
        print(
            "[WARN] 未找到RGL系统插件目录，自动切换为gpu_lidar: "
            f"{rgl_system_plugin or '(empty)'}"
        )
        use_rgl = False
    if use_rgl and not os.path.isfile(pattern_file_path):
        print(
            "[WARN] 未找到Livox Mid360 pattern文件，自动切换为gpu_lidar: "
            f"{pattern_file_path}"
        )
        use_rgl = False

    print(f"[INFO] 配置文件: {config_path}")
    print(f"[INFO] lidar_mode设置: {sim_config['lidar_mode']}")
    print(f"[INFO] 检测到NVIDIA显卡: {has_nvidia}")
    print(f"[INFO] 雷达仿真模式: {'RGL (NVIDIA)' if use_rgl else 'gpu_lidar (通用)'}")

    set_gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=_join_env_path(
            "GZ_SIM_RESOURCE_PATH",
            os.path.join(pkg_share_path, "models"),
        ),
    )
    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=_join_env_path(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            rgl_system_plugin if use_rgl else "",
            "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins",
        ),
    )
    set_gz_gui_plugin_path = SetEnvironmentVariable(
        name="GZ_GUI_PLUGIN_PATH",
        value=_join_env_path(
            "GZ_GUI_PLUGIN_PATH",
            rgl_gui_plugin if use_rgl and os.path.isdir(rgl_gui_plugin) else "",
        ),
    )

    rgl_patterns_dir = ""
    if use_rgl:
        rgl_patterns_dir = os.environ.get("RGL_PATTERNS_DIR", patterns_dir_fallback)
        if rgl_patterns_dir and not os.path.isdir(rgl_patterns_dir):
            print(
                "[WARN] RGL_PATTERNS_DIR不存在，将跳过该环境变量: "
                f"{rgl_patterns_dir}"
            )
            rgl_patterns_dir = ""
    set_rgl_patterns_dir = (
        SetEnvironmentVariable(name="RGL_PATTERNS_DIR", value=rgl_patterns_dir)
        if rgl_patterns_dir
        else None
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )

    if not use_rgl:
        world_file = _write_fallback_world(world_file)
    robot_model_path = _write_runtime_model(
        robot_model_path,
        sim_config["livox_mid360"],
        use_rgl,
        pattern_file_path,
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r -v 4 {world_file}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file",
            robot_model_path,
            "-name",
            "mecanum_bot",
            "-x",
            "3.4",
            "-y",
            "9.5",
            "-z",
            "0.3",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
        output="screen",
    )

    # Bridge for clock (required for use_sim_time)
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Bridge for cmd_vel topic
    cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
        output="screen",
    )

    if use_rgl:
        lidar_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/livox/lidar@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
            ],
            output="screen",
        )
    else:
        lidar_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="lidar_bridge_node",
            arguments=[
                "/gz_lidar_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
            ],
            remappings=[("/gz_lidar_points/points", "/livox/lidar")],
            output="screen",
        )

    lidar_frame_transform = None
    if not use_rgl:
        lidar_frame_transform = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--roll",
                "0",
                "--pitch",
                "0",
                "--yaw",
                "0",
                "--frame-id",
                "livox_lidar",
                "--child-frame-id",
                "mecanum_bot/livox_lidar/livox_lidar",
            ],
            output="screen",
        )

    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/livox/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"],
        output="screen",
    )

    chassis_imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/chassis/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"],
        output="screen",
    )

    gimbal_yaw_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/gimbal/yaw/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double"],
        output="screen",
    )

    gimbal_pitch_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/gimbal/pitch/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double"],
        output="screen",
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/camera@sensor_msgs/msg/Image@gz.msgs.Image"],
        output="screen",
    )

    pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/mecanum_bot/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose"
        ],
        output="screen",
    )

    joint_state_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model"],
        output="screen",
    )

    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])

    launch_actions = [
        set_gazebo_resource_path,
        set_gz_plugin_path,
        set_gz_gui_plugin_path,
        declare_use_sim_time,
        gazebo_launch,
        robot_state_publisher,
        delayed_spawn,
        clock_bridge,
        cmd_vel_bridge,
        lidar_bridge,
        imu_bridge,
        chassis_imu_bridge,
        gimbal_yaw_bridge,
        gimbal_pitch_bridge,
        camera_bridge,
        pose_bridge,
        joint_state_bridge,
    ]

    if set_rgl_patterns_dir is not None:
        launch_actions.insert(3, set_rgl_patterns_dir)
    if lidar_frame_transform is not None:
        launch_actions.append(lidar_frame_transform)

    return LaunchDescription(launch_actions)
