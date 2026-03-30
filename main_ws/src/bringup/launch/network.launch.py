import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    # bringupパッケージのshareディレクトリから設定ファイルの絶対パスを動的に取得
    bringup_dir = get_package_share_directory("bringup")
    # zenohd (router daemon) 用の設定: mode=router, listen quic/0.0.0.0:7447
    zenoh_router_config = os.path.join(bringup_dir, "config", "zenoh_router.json5")
    # ロボット側 ROS2 ノード用の設定: mode=client, connect 127.0.0.1:7447
    # zenohd と同じ router config を使うとポート競合するため別ファイルを使う
    zenoh_robot_config = os.path.join(bringup_dir, "config", "zenoh_robot.json5")

    return LaunchDescription(
        [
            # ==========================================
            # 1. RMW (ミドルウェア) の環境変数を強制上書き
            # ==========================================
            # このLaunchファイル（およびこれをincludeした親Launch）から
            # 起動されるすべてのROS 2ノードは、強制的にZenohプロトコルで通信します。
            # ROS2ノードは zenohd (client として localhost:7447 に接続) を使う。
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_zenoh_cpp"),
            SetEnvironmentVariable(
                "RMW_ZENOH_CONFIG_URI", f"file://{zenoh_robot_config}"
            ),
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
    )
