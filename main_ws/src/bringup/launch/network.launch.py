import os
import shlex
import subprocess
import tempfile

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)


def _build_network_actions(context):
    # bringupパッケージのshareディレクトリから設定ファイルの絶対パスを動的に取得
    bringup_dir = get_package_share_directory("bringup")
    actions = []

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

    if patched_ld_library_path:
        actions.append(
            SetEnvironmentVariable("LD_LIBRARY_PATH", patched_ld_library_path)
        )

    zenoh_router_config = os.path.join(bringup_dir, "config", "zenoh_router.json5")
    zenoh_robot_config = os.path.join(bringup_dir, "config", "zenoh_robot.json5")

    if not os.path.isfile(zenoh_router_config) or not os.path.isfile(zenoh_robot_config):
        raise RuntimeError("[network.launch] zenoh HA config files are missing")

    # ~/.ros/zenoh_quic/ に置くことでオペレーター側から scp しやすくする
    quic_dir = os.path.join(os.path.expanduser("~"), ".ros", "zenoh_quic")
    cert = os.path.join(quic_dir, "server.crt")
    key = os.path.join(quic_dir, "server.key")
    os.makedirs(quic_dir, exist_ok=True)

    quic_available = True
    if not os.path.isfile(cert) or not os.path.isfile(key):
        try:
            subprocess.run(
                [
                    "openssl",
                    "req",
                    "-x509",
                    "-newkey",
                    "rsa:2048",
                    "-sha256",
                    "-nodes",
                    "-days",
                    "3650",
                    "-subj",
                    "/CN=roborescue-zenoh-quic",
                    "-keyout",
                    key,
                    "-out",
                    cert,
                ],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            actions.append(LogInfo(msg="[network.launch] generated self-signed QUIC TLS cert/key"))
        except Exception as e:
            quic_available = False
            actions.append(
                LogInfo(
                    msg=(
                        "[network.launch] failed to prepare QUIC TLS assets; "
                        "router will run TCP-only for availability: "
                        + str(e)
                    )
                )
            )

    fd, tcp_only_cfg = tempfile.mkstemp(prefix="zenoh_router_tcp_fallback_", suffix=".json5")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(
            '{\n'
            '  mode: "router",\n'
            '  listen: { endpoints: ["tcp/0.0.0.0:7447"] },\n'
            '  scouting: { multicast: { enabled: false } },\n'
            '  transport: { shared_memory: { enabled: true } }\n'
            '}\n'
        )

    if quic_available:
        # QUIC 用: 証明書の絶対パスを注入して動的生成する
        # (zenoh_router.json5 は相対パスで書かれているため cwd 依存になるのを避ける)
        fd2, quic_cfg = tempfile.mkstemp(prefix="zenoh_router_quic_", suffix=".json5")
        with os.fdopen(fd2, "w", encoding="utf-8") as f:
            f.write(
                '{\n'
                '  mode: "router",\n'
                '  listen: { endpoints: ["quic/0.0.0.0:7447", "tcp/0.0.0.0:7447"] },\n'
                '  scouting: { multicast: { enabled: false } },\n'
                '  transport: {\n'
                '    shared_memory: { enabled: true },\n'
                '    link: {\n'
                '      tls: {\n'
                f'        listen_private_key: "{key}",\n'
                f'        listen_certificate: "{cert}"\n'
                '      }\n'
                '    }\n'
                '  }\n'
                '}\n'
            )
        actions.append(LogInfo(msg=f"[network.launch] QUIC cert for operator: scp robot@<IP>:{cert} ./quic/server.crt"))
        router_config_for_launch = quic_cfg
    else:
        router_config_for_launch = tcp_only_cfg

    router_cmd = (
        "while true; do "
        "  if ss -ltn '( sport = :7447 )' | grep -q ':7447'; then "
        "    if ss -ltnp '( sport = :7447 )' 2>/dev/null | grep -q 'zenohd'; then "
        "      sleep 3; "
        "      continue; "
        "    fi; "
        "    echo '[network.launch] :7447 busy by non-zenoh process, launching local zenohd on :17447'; "
        "    _tmp_cfg=$(mktemp); "
        "    sed 's#:7447#:17447#g' "
        + shlex.quote(router_config_for_launch)
        + " > \"$_tmp_cfg\"; "
        "    if ! zenohd --config \"$_tmp_cfg\"; then "
        "      echo '[network.launch] zenohd config failed on :17447; retrying TCP-only profile'; "
        "      _tmp_tcp=$(mktemp); "
        "      sed 's#:7447#:17447#g' "
        + shlex.quote(tcp_only_cfg)
        + " > \"$_tmp_tcp\"; "
        "      zenohd --config \"$_tmp_tcp\" || true; "
        "      rm -f \"$_tmp_tcp\"; "
        "    fi; "
        "    rm -f \"$_tmp_cfg\"; "
        "    echo '[network.launch] local zenohd(:17447) exited, retrying in 2s'; "
        "    sleep 2; "
        "    continue; "
        "  fi; "
        "  echo '[network.launch] launching local zenohd (QUIC preferred, TCP fallback) on :7447'; "
        "  if ! zenohd --config "
        + shlex.quote(router_config_for_launch)
        + "; then "
        "    echo '[network.launch] zenohd config failed on :7447; retrying TCP-only profile'; "
        "    zenohd --config "
        + shlex.quote(tcp_only_cfg)
        + " || true; "
        "  fi; "
        "  echo '[network.launch] local zenohd(:7447) exited, retrying in 2s'; "
        "  sleep 2; "
        "done"
    )

    actions.extend(
        [
        # ==========================================
        # 1. RMW (ミドルウェア) の環境変数を強制上書き
        # ==========================================
        # このLaunchファイル（およびこれをincludeした親Launch）から
        # 起動されるすべてのROS 2ノードは、強制的にZenohプロトコルで通信します。
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_zenoh_cpp"),
        # ルーター起動直後でも接続待ちを続け、初期化時の取りこぼしを減らす。
        SetEnvironmentVariable("ZENOH_ROUTER_CHECK_ATTEMPTS", "-1"),
        # ROS2ノードは zenohd (client として localhost:7447 に接続) を使う。
        SetEnvironmentVariable("RMW_ZENOH_CONFIG_URI", f"file://{zenoh_robot_config}"),
        # ==========================================
        # 2. Zenohルーター (zenohd) のデーモン起動
        # ==========================================
        # QUIC優先/TCPフォールバック構成でルーターを起動する。
        # SHM は zenohd 側でも有効にする必要があるため --config で渡す。
        ExecuteProcess(
            cmd=["bash", "-lc", router_cmd],
            output="screen",
            respawn=False,
            name="zenoh_router_daemon",
            cwd=os.path.join(bringup_dir, "config"),
        ),
        ]
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=_build_network_actions),
        ]
    )
