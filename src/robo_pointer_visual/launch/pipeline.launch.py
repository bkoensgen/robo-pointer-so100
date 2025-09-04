from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Config YAML path
    params_yaml = PathJoinSubstitution([
        FindPackageShare('robo_pointer_visual'),
        'config',
        'params.yaml',
    ])
    # Vision args
    yolo_model = LaunchConfiguration('yolo_model')
    camera_index = LaunchConfiguration('camera_index')
    confidence = LaunchConfiguration('confidence_threshold')
    flip_code = LaunchConfiguration('flip_code')
    frame_width = LaunchConfiguration('frame_width')
    frame_height = LaunchConfiguration('frame_height')
    frame_rate = LaunchConfiguration('frame_rate')
    publish_rate = LaunchConfiguration('publish_rate_hz')
    video_fourcc = LaunchConfiguration('video_fourcc')
    camera_backend = LaunchConfiguration('camera_backend')
    target_class_name = LaunchConfiguration('target_class_name')
    device = LaunchConfiguration('device')
    target_topic = LaunchConfiguration('target_topic')
    leader_arm_port = LaunchConfiguration('leader_arm_port')

    # Control args
    scale_x = LaunchConfiguration('pixel_to_cartesian_scale_x')
    scale_y = LaunchConfiguration('pixel_to_cartesian_scale_y')
    joint_states_topic = LaunchConfiguration('joint_states_topic')
    target_joint_angles_topic = LaunchConfiguration('target_joint_angles_topic')

    # Robot interface args
    read_freq = LaunchConfiguration('read_frequency_hz')
    enable_interface = LaunchConfiguration('enable_interface')
    publish_static_tf = LaunchConfiguration('publish_static_tf')
    tf_parent = LaunchConfiguration('tf_parent_frame')
    tf_child = LaunchConfiguration('tf_child_frame')
    tf_x = LaunchConfiguration('tf_x')
    tf_y = LaunchConfiguration('tf_y')
    tf_z = LaunchConfiguration('tf_z')
    tf_roll = LaunchConfiguration('tf_roll')
    tf_pitch = LaunchConfiguration('tf_pitch')
    tf_yaw = LaunchConfiguration('tf_yaw')

    return LaunchDescription([
        # Decls
        DeclareLaunchArgument('yolo_model', default_value='yolov8n.pt'),
        DeclareLaunchArgument('camera_index', default_value='/dev/video0'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.5'),
        DeclareLaunchArgument('flip_code', default_value='99'),
        DeclareLaunchArgument('frame_width', default_value='640'),
        DeclareLaunchArgument('frame_height', default_value='480'),
        DeclareLaunchArgument('frame_rate', default_value='30.0'),
        DeclareLaunchArgument('publish_rate_hz', default_value='15.0'),
        DeclareLaunchArgument('video_fourcc', default_value='MJPG'),
        DeclareLaunchArgument('camera_backend', default_value='v4l2'),
        DeclareLaunchArgument('target_class_name', default_value='bottle'),
        DeclareLaunchArgument('device', default_value='auto'),
        DeclareLaunchArgument('target_topic', default_value='detected_target_point'),
        DeclareLaunchArgument('leader_arm_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('pixel_to_cartesian_scale_x', default_value='0.0001'),
        DeclareLaunchArgument('pixel_to_cartesian_scale_y', default_value='0.0001'),
        DeclareLaunchArgument('read_frequency_hz', default_value='20.0'),
        DeclareLaunchArgument('joint_states_topic', default_value='joint_states'),
        DeclareLaunchArgument('target_joint_angles_topic', default_value='target_joint_angles'),
        DeclareLaunchArgument('enable_interface', default_value='true'),
        DeclareLaunchArgument('publish_static_tf', default_value='false'),
        DeclareLaunchArgument('tf_parent_frame', default_value='wrist_link'),
        DeclareLaunchArgument('tf_child_frame', default_value='camera_frame'),
        DeclareLaunchArgument('tf_x', default_value='0.0'),
        DeclareLaunchArgument('tf_y', default_value='0.0'),
        DeclareLaunchArgument('tf_z', default_value='0.0'),
        DeclareLaunchArgument('tf_roll', default_value='0.0'),
        DeclareLaunchArgument('tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('tf_yaw', default_value='0.0'),

        # Nodes
        Node(
            package='robo_pointer_visual',
            executable='vision_node',
            name='vision_node',
            parameters=[
                params_yaml,
                {
                    'yolo_model': yolo_model,
                    'camera_index': camera_index,
                    'confidence_threshold': confidence,
                    'flip_code': flip_code,
                    'frame_width': frame_width,
                    'frame_height': frame_height,
                    'frame_rate': frame_rate,
                    'publish_rate_hz': publish_rate,
                    'video_fourcc': video_fourcc,
                    'camera_backend': camera_backend,
                    'target_class_name': target_class_name,
                    'device': device,
                    'target_topic': target_topic,
                }
            ]
        ),
        Node(
            package='robo_pointer_visual',
            executable='robot_controller_node',
            name='robot_controller_node',
            parameters=[
                params_yaml,
                {
                    'pixel_to_cartesian_scale_x': scale_x,
                    'pixel_to_cartesian_scale_y': scale_y,
                    'target_topic': target_topic,
                    'joint_states_topic': joint_states_topic,
                    'target_joint_angles_topic': target_joint_angles_topic,
                }
            ]
        ),
        Node(
            package='robo_pointer_visual',
            executable='real_robot_interface',
            name='real_robot_interface',
            condition=IfCondition(enable_interface),
            parameters=[{
                'read_frequency_hz': read_freq,
                'leader_arm_port': leader_arm_port,
                'joint_states_topic': joint_states_topic,
                'target_joint_angles_topic': target_joint_angles_topic,
            }]
        ),
        # Optional static TF for RViz visualization (Euler angles order: roll pitch yaw)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_camera_tf',
            condition=IfCondition(publish_static_tf),
            arguments=[tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw, tf_parent, tf_child]
        ),
    ])
