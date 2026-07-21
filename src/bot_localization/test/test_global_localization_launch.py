from pathlib import Path
import ast


LAUNCH_FILE = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "global_localization.launch.py"
)


def test_global_localization_launch_is_valid_python():
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    function_names = {
        node.name for node in tree.body if isinstance(node, ast.FunctionDef)
    }
    assert "generate_launch_description" in function_names


def test_global_localization_launch_uses_lifecycle_manager_contract():
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert 'executable="lifecycle_manager"' in source
    assert '"node_names": lifecycle_nodes' in source
    assert 'executable="map_server"' in source
    assert "amcl_config" in source
