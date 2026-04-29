"""
Launch G1 Visual Backbone Policy — Residual Action.

Runs two ONNX policies (HLC + WBC) with visual embedding input from Theia.

Usage:
    ros2 launch cpp_control g1_visual_backbone_residual.launch.py
    ros2 launch cpp_control g1_visual_backbone_residual.launch.py onnx_model_path:=/path/to/hlc.onnx
    ros2 launch cpp_control g1_visual_backbone_residual.launch.py wandb_path:=entity/project/run_id
    ros2 launch cpp_control g1_visual_backbone_residual.launch.py wandb_path:=entity/project/run_id/model_1500.pt
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


WANDB_CACHE_ROOT = os.path.expanduser('~/.cache/cpp_control/wandb')


def _get_nested_cfg(cfg, dotted_key: str, default=None):
    """Walk a (possibly nested) wandb run.config to fetch a dotted key.

    Wandb stores configs sometimes as nested dicts, sometimes flattened with
    dots in the key. Try both."""
    try:
        as_dict = dict(cfg)
    except Exception:
        as_dict = cfg
    # 1. flattened: "env_cfg.runner_cfg.experiment_name" as a literal key
    if isinstance(as_dict, dict) and dotted_key in as_dict:
        return as_dict[dotted_key]
    # 2. nested walk
    cur = as_dict
    for part in dotted_key.split('.'):
        if isinstance(cur, dict) and part in cur:
            cur = cur[part]
        else:
            return default
    return cur


def _resolve_wandb_hlc(wandb_path: str) -> str:
    """Fetch `exported/policy.onnx` from the wandb run at `wandb_path` and
    return the absolute local path. Cached under ~/.cache/cpp_control/wandb/<run_id>/.
    Also prints a pretty validation banner with run path + experiment_name."""
    import wandb  # local import so non-wandb users aren't forced to install it

    # tolerate trailing /exported/policy.onnx-style suffixes by stripping back
    # to a 3-segment entity/project/run_id
    parts = [p for p in wandb_path.split('/') if p]
    run_path = '/'.join(parts[:3])

    api = wandb.Api()
    run = api.run(run_path)

    fname = 'exported/policy.onnx'
    run_id = run_path.split('/')[-1]
    cache_dir = os.path.join(WANDB_CACHE_ROOT, run_id)
    os.makedirs(cache_dir, exist_ok=True)
    local_path = os.path.join(cache_dir, fname)

    if not os.path.exists(local_path):
        print(f"[launch] downloading {run_path}/{fname} -> {local_path}")
        run.file(fname).download(root=cache_dir, replace=True)
    else:
        print(f"[launch] using cached wandb model: {local_path}")

    experiment_name = (
        _get_nested_cfg(run.config, 'env_cfg.runner_cfg.experiment_name')
        or _get_nested_cfg(run.config, 'runner_cfg.experiment_name')
        or _get_nested_cfg(run.config, 'experiment_name')
        or '<not found in run.config>'
    )

    banner = (
        "\n"
        "  ┌─────────────────────────── wandb policy loaded ───────────────────────────\n"
        f"  │  run path        : {run_path}\n"
        f"  │  experiment_name : {experiment_name}\n"
        f"  │  local onnx      : {local_path}\n"
        "  └────────────────────────────────────────────────────────────────────────────\n"
    )
    print(banner)

    return local_path


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')

    default_config = os.path.join(pkg, 'config', 'visual_backbone_policies', 'g1_residual.yaml')

    # Read ONNX paths from config
    with open(default_config, 'r') as f:
        config = yaml.safe_load(f)

    hlc_model = config.get('onnx_path', '')
    if hlc_model:
        hlc_model = os.path.join(pkg, 'models', hlc_model)

    wbc_model = config.get('wbc_onnx_path', '')
    if wbc_model:
        wbc_model = os.path.join(pkg, 'models', wbc_model)

    # wandb override for the HLC: if set, download (or hit cache) and use that path
    wandb_path_env = os.environ.get('WANDB_PATH', '')
    # also honor wandb_path:=... by peeking at sys.argv (launch args arrive as KEY:=VAL)
    import sys
    wandb_path_cli = ''
    for a in sys.argv:
        if a.startswith('wandb_path:='):
            wandb_path_cli = a.split(':=', 1)[1]
            break
    wandb_path = wandb_path_cli or wandb_path_env
    if wandb_path:
        hlc_model = _resolve_wandb_hlc(wandb_path)

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_config),
        DeclareLaunchArgument('wandb_path', default_value=wandb_path,
                              description='Optional wandb run path (entity/project/run_id[/model_xxx.onnx]). '
                                          'If set, overrides onnx_model_path with the cached download.'),
        DeclareLaunchArgument('onnx_model_path', default_value=hlc_model,
                              description='HLC policy ONNX (auto-filled from wandb_path if provided)'),
        DeclareLaunchArgument('wbc_onnx_model_path', default_value=wbc_model,
                              description='WBC policy ONNX (textop)'),
        DeclareLaunchArgument('embedding_topic', default_value='/theia_embedding'),
        DeclareLaunchArgument('object_goal_topic', default_value='/object_goal'),
        DeclareLaunchArgument('motion_topic', default_value='/tracker/motion'),

        Node(
            package='cpp_control',
            executable='g1_vbp_residual_node',
            name='g1_vbp_residual_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'onnx_model_path': LaunchConfiguration('onnx_model_path'),
                'wbc_onnx_model_path': LaunchConfiguration('wbc_onnx_model_path'),
                'embedding_topic': LaunchConfiguration('embedding_topic'),
                'object_goal_topic': LaunchConfiguration('object_goal_topic'),
                'motion_topic': LaunchConfiguration('motion_topic'),
            }],
        ),
    ])
