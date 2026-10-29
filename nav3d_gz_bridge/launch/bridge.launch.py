import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_bridge(context):
    uav_name = LaunchConfiguration("uav_name").perform(context)
    odom_topic = f"/model/{uav_name}/odometry"
    use_sim_time = {"use_sim_time": True}

    return [
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--qx",
                "0",
                "--qy",
                "0",
                "--qz",
                "0",
                "--qw",
                "1",
                "--frame-id",
                "map",
                "--child-frame-id",
                "odom",
            ],
            parameters=[use_sim_time],
        ),
        launch_ros.actions.Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
            parameters=[use_sim_time],
        ),
        launch_ros.actions.Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[f"{odom_topic}@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
            remappings=[(odom_topic, "/odom")],
            output="screen",
            parameters=[use_sim_time],
        ),
        launch_ros.actions.Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image"],
            remappings=[("/depth_camera", "/camera/depth/image_raw")],
            output="screen",
            parameters=[use_sim_time],
        ),
        launch_ros.actions.Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
            remappings=[("/camera_info", "/camera/camera_info")],
            output="screen",
            parameters=[use_sim_time],
        ),
        launch_ros.actions.Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
            ],
            remappings=[("/depth_camera/points", "/camera/depth/points")],
            output="screen",
            parameters=[
                use_sim_time,
                {
                    # Gazebo Harmonic labels the cloud optical although its XYZ is X-forward.
                    # Correct only the frame_id; TF rotates oakd_depth_frame to the optical frame.
                    "override_frame_id": "oakd_depth_frame",
                    "qos_overrides./camera/depth/points.publisher.reliability": "best_effort",
                    "qos_overrides./camera/depth/points.publisher.history": "keep_last",
                    "qos_overrides./camera/depth/points.publisher.depth": 1,
                },
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("uav_name", default_value="x500"),
            OpaqueFunction(function=_launch_bridge),
        ]
    )
