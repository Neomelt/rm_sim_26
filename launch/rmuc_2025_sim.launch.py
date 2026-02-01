import os
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
    def _has_nvidia_ml():
        candidates = [
            "/usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1",
            "/usr/lib64/libnvidia-ml.so.1",
            "/usr/lib/libnvidia-ml.so.1",
        ]
        return any(os.path.exists(p) for p in candidates)

    def _write_fallback_world(src_path):
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

    def _write_fallback_model(src_path):
        with open(src_path, "r", encoding="utf-8") as f:
            content = f.read()
        sensor_start = content.find('<sensor name="RGLLidar" type="custom">')
        if sensor_start != -1:
            sensor_end = content.find("</sensor>", sensor_start)
            if sensor_end != -1:
                # Livox Mid360 parameters from pb2025_robot_description:
                # Horizontal: 360° (0 to 2*pi), samples=1875 at 20Hz or proportional
                # Vertical: -7° to +52° (-0.122 to 0.908 rad), 32 layers
                # Range: 0.1m to 40m
                fallback_sensor = (
                    '<sensor name="livox_lidar" type="gpu_lidar">\n'
                    "  <pose>0 0 0 0 0 0</pose>\n"
                    "  <always_on>true</always_on>\n"
                    "  <visualize>true</visualize>\n"
                    "  <update_rate>10</update_rate>\n"
                    "  <topic>livox/lidar</topic>\n"
                    "  <ignition_frame_id>livox_lidar</ignition_frame_id>\n"
                    "  <ray>\n"
                    "    <scan>\n"
                    "      <horizontal>\n"
                    "        <samples>1875</samples>\n"
                    "        <resolution>1.0</resolution>\n"
                    "        <min_angle>0</min_angle>\n"
                    "        <max_angle>6.2831852</max_angle>\n"
                    "      </horizontal>\n"
                    "      <vertical>\n"
                    "        <samples>32</samples>\n"
                    "        <min_angle>-0.12217304764</min_angle>\n"
                    "        <max_angle>0.90757121104</max_angle>\n"
                    "      </vertical>\n"
                    "    </scan>\n"
                    "    <range>\n"
                    "      <min>0.1</min>\n"
                    "      <max>40.0</max>\n"
                    "    </range>\n"
                    "    <noise>\n"
                    "      <type>gaussian</type>\n"
                    "      <mean>0.0</mean>\n"
                    "      <stddev>0.01</stddev>\n"
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

    # Package name
    package_name = "rm_sim_26"

    has_nvidia = _has_nvidia_ml()

    # Get package share directory
    pkg_share = FindPackageShare(package_name)
    pkg_share_path = get_package_share_directory(package_name)

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

    # Optional RGL plugin build paths (can be overridden by env vars)
    rgl_root = os.environ.get("RGL_GZ_PLUGIN_ROOT", source_root) if has_nvidia else ""
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
    if rgl_system_plugin and has_nvidia:
        system_plugin_path_parts += [":", rgl_system_plugin]
    system_plugin_path_parts += [":", "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins"]

    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=system_plugin_path_parts,
    )

    gui_plugin_path_parts = [os.environ.get("GZ_GUI_PLUGIN_PATH", "")]
    if rgl_gui_plugin and has_nvidia:
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
        os.environ.get("RGL_PATTERNS_DIR", patterns_dir_fallback) if has_nvidia else ""
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

    # Override to non-RGL files when NVIDIA runtime is not available
    if not has_nvidia:
        world_file = _write_fallback_world(
            os.path.join(pkg_share_path, "worlds", "rmuc_2025_world.world")
        )
        robot_model_path = _write_fallback_model(
            os.path.join(pkg_share_path, "moudles", "mecanum_car", "model.sdf")
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
    if has_nvidia:
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
        # Bridge directly to /livox/lidar/points
        lidar_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/livox/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
            ],
            output="screen",
        )

    # LaserScan to PointCloud converter no longer needed since gpu_lidar outputs PointCloud2 directly
    laserscan_to_cloud = None

    # Static TF publisher to remap Gazebo's frame to the robot model frame
    # Gazebo gpu_lidar uses full model path as frame_id: mecanum_bot/livox_lidar/livox_lidar
    # We need to link it to the robot's livox_lidar frame with identity transform
    lidar_frame_remap = None
    if not has_nvidia:
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
