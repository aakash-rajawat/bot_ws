from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    workspace_root = Path("/workspaces/bot_ws")
    venv_dir = workspace_root / "xfeat_lightglue"
    accelerated_features_dir = workspace_root / "third_party" / "accelerated_features"
    venv_python = venv_dir / "bin" / "python"
    server_script = (
        workspace_root
        / "src"
        / "bot_vision_py"
        / "bot_vision_py"
        / "xfeat_lightglue_server.py"
    )

    xfeat_lightglue_server = ExecuteProcess(
        cmd=[
            str(venv_python),
            str(server_script),
        ],
        additional_env={
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "VIRTUAL_ENV": str(venv_dir),
            "PATH": f"{venv_dir / 'bin'}:{os.environ.get('PATH', '')}",
            "PYTHONPATH": f"{accelerated_features_dir}:{os.environ.get('PYTHONPATH', '')}",
            "PYTHONUNBUFFERED": "1",
        },
        output="screen",
    )

    return LaunchDescription([
        xfeat_lightglue_server,
    ])
