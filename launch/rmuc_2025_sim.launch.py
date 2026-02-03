import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ==================== 辅助函数 ====================
    def _has_nvidia_ml():
        """检测系统是否有NVIDIA显卡驱动"""
        candidates = [
            "/usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1",
            "/usr/lib64/libnvidia-ml.so.1",
            "/usr/lib/libnvidia-ml.so.1",
        ]
        return any(os.path.exists(p) for p in candidates)

    def _load_sim_config(config_path):
        """加载仿真配置文件"""
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
            }
        }
        if os.path.exists(config_path):
            with open(config_path, "r", encoding="utf-8") as f:
                user_config = yaml.safe_load(f) or {}
            # 合并配置
            for key in default_config:
                if key in user_config:
                    if isinstance(default_config[key], dict):
                        default_config[key].update(user_config[key])
                    else:
                        default_config[key] = user_config[key]
        return default_config

    def _determine_use_rgl(lidar_mode, has_nvidia):
        """根据配置和硬件情况决定是否使用RGL"""
        if lidar_mode == "rgl":
            if not has_nvidia:
                print("[WARN] lidar_mode设置为'rgl'但未检测到NVIDIA显卡，强制切换为gpu_lidar")
                return False
            return True
        elif lidar_mode == "gpu_lidar":
            return False
        else:  # auto
            return has_nvidia

    def _write_fallback_world(src_path):
        """生成不含RGL插件的world文件"""
        with open(src_path, "r", encoding="utf-8") as f:
            content = f.read()
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
        out_path = "/tmp/rm_sim_26_world_no_rgl.world"
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(content)
        return out_path

    def _write_fallback_model(src_path, lidar_config):
        """生成使用gpu_lidar的model文件"""
        with open(src_path, "r", encoding="utf-8") as f:
            content = f.read()
        sensor_start = content.find('<sensor name="RGLLidar" type="custom">')
        if sensor_start != -1:
            sensor_end = content.find("</sensor>", sensor_start)
            if sensor_end != -1:
                # 从配置文件读取Livox Mid360参数
                fallback_sensor = (
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
                content = (
                    content[:sensor_start]
                    + fallback_sensor
                    + content[sensor_end + len("</sensor>") :]
                )
        out_path = "/tmp/rm_sim_26_model_no_rgl.sdf"
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(content)
        return out_path

    # ==================== 主要逻辑 ====================
    # Package name
    package_name = "rm_sim_26"

    # Get package share directory
    pkg_share = FindPackageShare(package_name)
    pkg_share_path = get_package_share_directory(package_name)

    # 加载配置文件
    config_path = os.path.join(pkg_share_path, "config", "sim_config.yaml")
    sim_config = _load_sim_config(config_path)

    # 检测硬件并决定使用哪种雷达模式
    has_nvidia = _has_nvidia_ml()
    use_rgl = _determine_use_rgl(sim_config["lidar_mode"], has_nvidia)
    
    print(f"[INFO] 配置文件: {config_path}")
    print(f"[INFO] lidar_mode设置: {sim_config['lidar_mode']}")
    print(f"[INFO] 检测到NVIDIA显卡: {has_nvidia}")
    print(f"[INFO] 雷达仿真模式: {'RGL (NVIDIA)' if use_rgl else 'gpu_lidar (通用)'}")

    # 设置Gazebo资源路径
    set_gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            ":",
            os.path.join(pkg_share_path, "moudles"),
        ],
    )

    # Detect repository root when running from source (optional)
    source_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    if not os.path.isdir(os.path.join(source_root, "external", "RGLGazeboPlugin")):
        source_root = ""

    # Optional RGL plugin build paths (can be overridden by env vars or config)
    rgl_root_from_config = sim_config.get("rgl_plugin_root", "")
    rgl_root = ""
    if use_rgl:
        rgl_root = os.environ.get("RGL_GZ_PLUGIN_ROOT", rgl_root_from_config or source_root)
    
    rgl_system_plugin = (
        os.path.join(rgl_root, "external", "RGLGazeboPlugin", "install", "RGLServerPlugin")
        if rgl_root
        else ""
    )
    rgl_gui_plugin = (
        os.path.join(rgl_root, "external", "RGLGazeboPlugin", "install", "RGLVisualize")
        if rgl_root
        else ""
    )

    system_plugin_path_parts = [os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")]
    if rgl_system_plugin and use_rgl:
        system_plugin_path_parts += [":", rgl_system_plugin]
    system_plugin_path_parts += [":", "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins"]

    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=system_plugin_path_parts,
    )

    gui_plugin_path_parts = [os.environ.get("GZ_GUI_PLUGIN_PATH", "")]
    if rgl_gui_plugin and use_rgl:
        gui_plugin_path_parts += [":", rgl_gui_plugin]
    set_gz_gui_plugin_path = SetEnvironmentVariable(
        name="GZ_GUI_PLUGIN_PATH",
        value=gui_plugin_path_parts,
    )

    patterns_dir_fallback = (
        os.path.join(rgl_root, "external", "RGLGazeboPlugin", "lidar_patterns")
        if rgl_root
        else ""
    )
    rgl_patterns_dir = (
        os.environ.get("RGL_PATTERNS_DIR", patterns_dir_fallback) if use_rgl else ""
    )
    set_rgl_patterns_dir = (
        SetEnvironmentVariable(name="RGL_PATTERNS_DIR", value=rgl_patterns_dir)
        if rgl_patterns_dir
        else None
    )

    # World file path (fallback to non-RGL on non-NVIDIA)
    world_file = PathJoinSubstitution([pkg_share, "worlds", "rmuc_2025_world.world"])

    robot_urdf_path = PathJoinSubstitution(
        [pkg_share, "moudles", "mecanum_car", "robot.urdf"]
    )

    # Robot model path (fallback to non-RGL on non-NVIDIA)
    robot_model_path = PathJoinSubstitution(
        [pkg_share, "moudles", "mecanum_car", "model.sdf"]
    )

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )

    # Override to non-RGL files when not using RGL
    if not use_rgl:
        world_file = _write_fallback_world(
            os.path.join(pkg_share_path, "worlds", "rmuc_2025_world.world")
        )
        robot_model_path = _write_fallback_model(
            os.path.join(pkg_share_path, "moudles", "mecanum_car", "model.sdf"),
            sim_config["livox_mid360"]
        )

    # Launch Gazebo with the world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": Command(["cat ", robot_urdf_path])},
        ],
    )

    # Spawn the robot in Gazebo
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
            "0.28",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
        output="screen",
    )

    # Bridge for cmd_vel topic
    cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
        output="screen",
    )

    # Bridge for lidar topic
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
        # gpu_lidar outputs PointCloud2 on <topic>/points
        # Bridge to internal topic, then relay with correct frame_id to /livox/lidar
        lidar_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/gz_lidar_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
            ],
            remappings=[
                ("/gz_lidar_points/points", "/livox/lidar"),
            ],
            output="screen",
        )

    # LaserScan to PointCloud converter no longer needed since gpu_lidar outputs PointCloud2 directly
    laserscan_to_cloud = None

    # Frame ID relay: republish pointcloud with correct frame_id
    # Gazebo gpu_lidar ignores gz_frame_id and uses model path as frame_id
    # Use a simple Python node to fix the frame_id in the pointcloud header
    lidar_frame_remap = None
    if not use_rgl:
        lidar_frame_remap = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "livox_lidar",
                "--child-frame-id", "mecanum_bot/livox_lidar/livox_lidar",
            ],
            output="screen",
        )

    # Bridge for IMU topic
    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/livox/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"],
        output="screen",
    )

    # Bridge for gimbal yaw control
    gimbal_yaw_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/gimbal/yaw/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double"],
        output="screen",
    )

    # Bridge for gimbal pitch control
    gimbal_pitch_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/gimbal/pitch/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double"],
        output="screen",
    )

    # Bridge for camera
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/camera@sensor_msgs/msg/Image@gz.msgs.Image"],
        output="screen",
    )

    # Bridge for pose (TF)
    pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/mecanum_bot/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose"
        ],
        output="screen",
    )

    # Bridge for joint states
    joint_state_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model"],
        output="screen",
    )

    # Delay the spawning of the robot to ensure Gazebo is ready
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])

    launch_actions = [
        set_gazebo_resource_path,
        set_gz_plugin_path,
        set_gz_gui_plugin_path,
        declare_use_sim_time,
        gazebo_launch,
        robot_state_publisher,
        delayed_spawn,
        cmd_vel_bridge,
        lidar_bridge,
        imu_bridge,
        gimbal_yaw_bridge,
        gimbal_pitch_bridge,
        camera_bridge,
        pose_bridge,
        joint_state_bridge,
    ]

    if set_rgl_patterns_dir is not None:
        launch_actions.insert(3, set_rgl_patterns_dir)

    if laserscan_to_cloud is not None:
        launch_actions.append(laserscan_to_cloud)

    if lidar_frame_remap is not None:
        launch_actions.append(lidar_frame_remap)

    return LaunchDescription(launch_actions)
