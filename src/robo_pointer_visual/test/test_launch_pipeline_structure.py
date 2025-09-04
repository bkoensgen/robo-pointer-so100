import types

def test_pipeline_launch_has_enable_interface_and_condition():
    # Import the launch module directly
    from robo_pointer_visual.launch import pipeline as pipeline_launch
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node
    from launch.conditions import IfCondition

    ld = pipeline_launch.generate_launch_description()
    assert isinstance(ld, LaunchDescription)

    # Collect entities (DeclareLaunchArgument and Node actions)
    entities = getattr(ld, 'entities', []) or []
    decls = [e for e in entities if isinstance(e, DeclareLaunchArgument)]
    nodes = [e for e in entities if isinstance(e, Node)]

    # Ensure enable_interface is declared
    has_enable_interface = any(getattr(d, 'name', '') == 'enable_interface' for d in decls)
    assert has_enable_interface, "enable_interface arg should be declared in pipeline.launch.py"

    # Find the real_robot_interface node and ensure it has an IfCondition
    iface_nodes = [n for n in nodes if getattr(n, 'name', None) == 'real_robot_interface']
    assert len(iface_nodes) == 1, "real_robot_interface node must be present in launch description"
    iface = iface_nodes[0]
    cond = getattr(iface, 'condition', None)
    assert isinstance(cond, IfCondition), "real_robot_interface must be guarded by IfCondition(enable_interface)"

