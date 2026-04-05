import os
import shlex
import subprocess
import tempfile
import shutil

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
    zenoh_robot_config = os.path.abspath(os.path.join(bringup_dir, "config", "zenoh_robot.json5"))

    # Certificates paths - Final fixed locations
    # 1. 実際の実行で使われる場所 (install内)
    project_quic_dir = os.path.abspath(os.path.join(bringup_dir, "config", "quic"))
    # 2. scp 用のソースディレクトリ
    src_quic_dir = os.path.abspath(os.path.join(bringup_dir, "..", "..", "..", "src", "bringup", "config", "quic"))
    
    os.makedirs(project_quic_dir, exist_ok=True)
    cert = os.path.join(project_quic_dir, "server.crt")
    key = os.path.join(project_quic_dir, "server.key")

    quic_available = True
    if not os.path.isfile(cert) or not os.path.isfile(key):
        try:
            # Generate a compatible End-Entity cert for QUIC (not a CA)
            with tempfile.NamedTemporaryFile(mode='w', suffix='.conf') as f:
                f.write("[req]\ndistinguished_name=req_distinguished_name\nx509_extensions=v3_req\nprompt=no\n")
                f.write("[req_distinguished_name]\nCN=roborescue-zenoh-quic\n")
                f.write("[v3_req]\nkeyUsage=critical,digitalSignature,keyEncipherment\nextendedKeyUsage=serverAuth\nbasicConstraints=critical,CA:FALSE\nsubjectAltName=DNS:localhost,IP:127.0.0.1,IP:10.42.0.1,IP:100.114.200.30\n")
                f.flush()
                
                subprocess.run(
                    [
                        "openssl", "req", "-x509", "-newkey", "rsa:2048", "-sha256", "-nodes", "-days", "3650",
                        "-keyout", key, "-out", cert, "-config", f.name
                    ],
                    check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
            actions.append(LogInfo(msg="[network.launch] generated high-compatibility QUIC TLS cert/key"))
            
            # ソースディレクトリへコピー (これで scp が通るようになる)
            if os.path.isdir(os.path.dirname(src_quic_dir)):
                os.makedirs(src_quic_dir, exist_ok=True)
                shutil.copy2(cert, os.path.join(src_quic_dir, "server.crt"))
                shutil.copy2(key, os.path.join(src_quic_dir, "server.key"))
                actions.append(LogInfo(msg=f"[network.launch] synced certs to {src_quic_dir} for scp"))
        except Exception as e:
            quic_available = False
            actions.append(LogInfo(msg=f"[network.launch] failed to prepare QUIC TLS assets: {e}"))

    # Router config (zenohd用)
    fd, tcp_only_cfg = tempfile.mkstemp(prefix="zenoh_router_tcp_fallback_", suffix=".json5")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write('{\n  mode: "router",\n  listen: { endpoints: ["tcp/0.0.0.0:7447"] },\n  scouting: { multicast: { enabled: false } },\n  transport: { shared_memory: { enabled: true } }\n}\n')

    if quic_available:
        fd2, quic_cfg = tempfile.mkstemp(prefix="zenoh_router_quic_", suffix=".json5")
        with os.fdopen(fd2, "w", encoding="utf-8") as f:
            f.write('{\n  mode: "router",\n  listen: { endpoints: ["quic/0.0.0.0:7447", "tcp/0.0.0.0:7447"] },\n  scouting: { multicast: { enabled: false } },\n  transport: {\n    shared_memory: { enabled: true },\n    link: {\n      tls: {\n' + f'        listen_private_key: "{key}",\n        listen_certificate: "{cert}"\n' + '      }\n    }\n  }\n}\n')
        actions.append(LogInfo(msg=f"[network.launch] QUIC cert for operator is ready at {src_quic_dir}/server.crt"))
        router_config_for_launch = quic_cfg
    else:
        router_config_for_launch = tcp_only_cfg

    router_cmd = (
        "pkill -9 -x zenohd || true; "
        "while true; do "
        "  zenohd --config " + shlex.quote(router_config_for_launch) + " || zenohd --config " + shlex.quote(tcp_only_cfg) + "; "
        "  sleep 2; "
        "done"
    )

    actions.extend([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_zenoh_cpp"),
        SetEnvironmentVariable("ZENOH_ROUTER_CHECK_ATTEMPTS", "-1"),
        SetEnvironmentVariable("ZENOH_SESSION_CONFIG_URI", zenoh_robot_config),
        ExecuteProcess(cmd=["bash", "-lc", router_cmd], output="screen", name="zenoh_router_daemon", cwd=bringup_dir),
    ])

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_build_network_actions)])
