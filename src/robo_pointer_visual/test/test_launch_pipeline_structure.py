import importlib.util
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node  # optional; type check not strictly required
from launch.conditions import IfCondition


def test_pipeline_launch_has_enable_interface_and_condition():
    # Load launch file as a module by path
    pipeline_path = Path(__file__).resolve().parent.parent / 'launch' / 'pipeline.launch.py'
    spec = importlib.util.spec_from_file_location('pipeline_launch', str(pipeline_path))
    pipeline_launch = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(pipeline_launch)

    ld = pipeline_launch.generate_launch_description()
    assert isinstance(ld, LaunchDescription)

    # Collect entities (DeclareLaunchArgument and Node actions)
    entities = getattr(ld, 'entities', []) or []
    # Flatten nested actions to be robust across launch versions
    def flatten_entities(items):
        out = []
        stack = list(items)
        while stack:
            a = stack.pop()
            out.append(a)
            children = getattr(a, 'entities', None)
            if children:
                stack.extend(children)
        return out

    decls = [e for e in entities if isinstance(e, DeclareLaunchArgument)]
    nodes = [e for e in flatten_entities(entities) if isinstance(e, Node)]

    # Ensure enable_interface and interface_type are declared
    has_enable_interface = any(getattr(d, 'name', '') == 'enable_interface' for d in decls)
    has_interface_type = any(getattr(d, 'name', '') == 'interface_type' for d in decls)
    assert has_enable_interface, "enable_interface arg should be declared in pipeline.launch.py"
    assert has_interface_type, "interface_type arg should be declared in pipeline.launch.py"

    # Look for interface nodes by combining attributes and repr
    def as_text(n):
        parts = [repr(n)]
        for attr in ('name', 'executable', '_Node__node_name', '_Node__executable'):
            v = getattr(n, attr, None)
            if v is not None:
                parts.append(str(v))
        return " ".join(parts)

    flat_repr = "\n".join(as_text(n) for n in nodes)
    has_real = ('real_robot_interface' in flat_repr)
    has_mock = ('mock_robot_interface' in flat_repr)
    assert has_real or has_mock, "At least one interface node (real or mock) must be present"
