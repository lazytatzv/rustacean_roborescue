import os
import shlex
import subprocess
import tempfile

import yaml
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
    bringup_dir = get_package_share_directory("bringup")
    actions = []

    # launch_config.yaml から quic_regenerate_cert フラグを読む
    launch_config_path = os.path.join(bringup_dir, "config", "launch_config.yaml")
    try:
        with open(launch_config_path) as f:
            launch_cfg = yaml.safe_load(f) or {}
    except Exception:
        launch_cfg = {}
    quic_regenerate_cert = bool(launch_cfg.get("quic_regenerate_cert", False))

    # LD_LIBRARY_PATH patch for Zenoh
    zenoh_lib_paths = []
    try:
        zenoh_prefix = get_package_prefix("rmw_zenoh_cpp")
        for d in ("lib", "lib64"):
            candidate = os.path.join(zenoh_prefix, d)
            if os.path.isdir(candidate):
                zenoh_lib_paths.append(candidate)
    except PackageNotFoundError:
        pass

    current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    ld_parts = [p for p in zenoh_lib_paths if p]
    if current_ld_library_path:
        ld_parts.append(current_ld_library_path)
    patched_ld_library_path = ":".join(ld_parts)

    if patched_ld_library_path:
        actions.append(SetEnvironmentVariable("LD_LIBRARY_PATH", patched_ld_library_path))

    # Config paths
    zenoh_robot_config = os.path.realpath(os.path.join(bringup_dir, "config", "zenoh_robot.json5"))

    # 自己署名証明書 (QUIC/TLS 必須のため生成するが、クライアント側は検証スキップ)
    # Tailscale (WireGuard) が実際の暗号化・認証を担うため CA チェーン不要。
    quic_dir = os.path.abspath(os.path.join(bringup_dir, "config", "quic"))
    os.makedirs(quic_dir, exist_ok=True)
    cert = os.path.join(quic_dir, "server.crt")
    key = os.path.join(quic_dir, "server.key")

    quic_available = True
    if quic_regenerate_cert:
        for f in (cert, key):
            try:
                os.remove(f)
            except FileNotFoundError:
                pass
        actions.append(LogInfo(msg="[network.launch] quic_regenerate_cert=true: regenerating cert"))
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
                    "-keyout",
                    key,
                    "-out",
                    cert,
                    "-subj",
                    "/CN=zenoh-robot",
                    "-addext",
                    "basicConstraints=critical,CA:FALSE",
                    "-addext",
                    "subjectAltName=DNS:localhost,IP:127.0.0.1,IP:10.42.0.1,IP:100.114.200.30",
                ],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            actions.append(LogInfo(msg="[network.launch] generated self-signed QUIC cert"))
        except Exception as e:
            quic_available = False
            actions.append(LogInfo(msg=f"[network.launch] failed to generate QUIC cert: {e}"))

    # Router config (zenohd 用)
    fd, tcp_only_cfg = tempfile.mkstemp(prefix="zenoh_router_tcp_fallback_", suffix=".json5")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(
            '{\n  mode: "router",\n  listen: { endpoints: ["tcp/0.0.0.0:7447"] },\n'
            "  scouting: { multicast: { enabled: false } },\n"
            "  transport: { shared_memory: { enabled: true } }\n}\n"
        )

    if quic_available:
        fd2, quic_cfg = tempfile.mkstemp(prefix="zenoh_router_quic_", suffix=".json5")
        with os.fdopen(fd2, "w", encoding="utf-8") as f:
            f.write(
                '{\n  mode: "router",\n'
                '  listen: { endpoints: ["quic/0.0.0.0:7447", "tcp/0.0.0.0:7447"] },\n'
                "  scouting: { multicast: { enabled: false } },\n"
                "  transport: {\n    shared_memory: { enabled: true },\n"
                "    link: {\n      tls: {\n"
                f'        listen_private_key: "{key}",\n'
                f'        listen_certificate: "{cert}"\n'
                "      }\n    }\n  }\n}\n"
            )
        router_config_for_launch = quic_cfg
    else:
        router_config_for_launch = tcp_only_cfg

    router_cmd = (
        "pkill -9 -x zenohd || true; "
        "while true; do "
        "  RUST_LOG=info "
        "  zenohd --config "
        + shlex.quote(router_config_for_launch)
        + " || zenohd --config "
        + shlex.quote(tcp_only_cfg)
        + "; "
        "  sleep 2; "
        "done"
    )

    actions.extend(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_zenoh_cpp"),
            SetEnvironmentVariable("ZENOH_ROUTER_CHECK_ATTEMPTS", "-1"),
            SetEnvironmentVariable("ZENOH_SESSION_CONFIG_URI", zenoh_robot_config),
            ExecuteProcess(
                cmd=["bash", "-lc", router_cmd],
                output="screen",
                name="zenoh_router_daemon",
                cwd=bringup_dir,
            ),
        ]
    )

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_build_network_actions)])
