import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    # bringupパッケージのshareディレクトリから設定ファイルの絶対パスを動的に取得
    bringup_dir = get_package_share_directory('bringup')
    zenoh_config_path = os.path.join(bringup_dir, 'config', 'zenoh_robot.json5')

    return LaunchDescription([
        # ==========================================
        # 1. RMW (ミドルウェア) の環境変数を強制上書き
        # ==========================================
        # このLaunchファイル（およびこれをincludeした親Launch）から
        # 起動されるすべてのROS 2ノードは、強制的にZenohプロトコルで通信します。
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp'),
        SetEnvironmentVariable('RMW_ZENOH_CONFIG_URI', f'file://{zenoh_config_path}'),

        # ==========================================
        # 2. Zenohルーター (zenohd) のデーモン起動
        # ==========================================
        # Nix環境に入っている zenohd バイナリを呼び出し、
        # ロボット実機のネットワーク基盤としてQUICプロトコルで待ち受けます。
        ExecuteProcess(
            cmd=[
                'zenohd', 
                '--listen', 'quic/0.0.0.0:7447', 
                '--no-multicast-scout' # Wi-Fi帯域を守るためのマルチキャスト無効化
            ],
            output='screen',
            respawn=True,         # 万が一プロセスが死んでも自動復帰（レスキュー用必須設定）
            respawn_delay=2.0,    # 再起動までのインターバル(秒)
            name='zenoh_router_daemon'
        )
    ])
