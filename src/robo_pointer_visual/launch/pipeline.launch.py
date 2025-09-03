from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    leader_arm_port = LaunchConfiguration('leader_arm_port')

    # Control args
    scale_x = LaunchConfiguration('pixel_to_cartesian_scale_x')
    scale_y = LaunchConfiguration('pixel_to_cartesian_scale_y')

    # Robot interface args
    read_freq = LaunchConfiguration('read_frequency_hz')

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
        DeclareLaunchArgument('leader_arm_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('pixel_to_cartesian_scale_x', default_value='0.0001'),
        DeclareLaunchArgument('pixel_to_cartesian_scale_y', default_value='0.0001'),
        DeclareLaunchArgument('read_frequency_hz', default_value='20.0'),

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
                }
            ]
        ),
        Node(
            package='robo_pointer_visual',
            executable='real_robot_interface',
            name='real_robot_interface',
            parameters=[{
                'read_frequency_hz': read_freq,
                'leader_arm_port': leader_arm_port,
            }]
        ),
    ])
