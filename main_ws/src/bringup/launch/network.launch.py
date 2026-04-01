import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    # bringupパッケージのshareディレクトリから設定ファイルの絶対パスを動的に取得
    bringup_dir = get_package_share_directory("bringup")
    # Use the router config for both zenohd and as a local RMW config
    # (previously zenoh_robot.json5 was used here; that file was removed)
    zenoh_router_config = os.path.join(bringup_dir, "config", "zenoh_router.json5")

    # Nix環境では rmw_zenoh_cpp の .so が LD_LIBRARY_PATH に
    # 自動で入らないケースがあるため、起動時に補完する。
    zenoh_lib_paths = []
    try:
        zenoh_prefix = get_package_prefix("rmw_zenoh_cpp")
        for d in ("lib", "lib64"):
            candidate = os.path.join(zenoh_prefix, d)
            if os.path.isdir(candidate):
                zenoh_lib_paths.append(candidate)
    except PackageNotFoundError:
        zenoh_lib_paths = []

    current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    ld_parts = [p for p in zenoh_lib_paths if p]
    if current_ld_library_path:
        ld_parts.append(current_ld_library_path)
    patched_ld_library_path = ":".join(ld_parts)

    actions = [
        # ==========================================
        # 1. RMW (ミドルウェア) の環境変数を強制上書き
        # ==========================================
        # このLaunchファイル（およびこれをincludeした親Launch）から
        # 起動されるすべてのROS 2ノードは、強制的にZenohプロトコルで通信します。
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_zenoh_cpp"),
        SetEnvironmentVariable("RMW_ZENOH_CONFIG_URI", f"file://{zenoh_router_config}"),
        # ==========================================
        # 2. Zenohルーター (zenohd) のデーモン起動
        # ==========================================
        # QUIC でリッスンし、オペレータ PC からの接続を受け付ける。
        # SHM は zenohd 側でも有効にする必要があるため --config で渡す。
        ExecuteProcess(
            cmd=[
                "zenohd",
                "--config",
                zenoh_router_config,
            ],
            output="screen",
            respawn=True,  # 万が一プロセスが死んでも自動復帰（レスキュー用必須設定）
            respawn_delay=2.0,
            name="zenoh_router_daemon",
        ),
    ]

    if patched_ld_library_path:
        actions.insert(
            0, SetEnvironmentVariable("LD_LIBRARY_PATH", patched_ld_library_path)
        )

    return LaunchDescription(actions)
