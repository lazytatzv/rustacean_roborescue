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
    zenoh_robot_config = os.path.realpath(os.path.join(bringup_dir, "config", "zenoh_robot.json5"))

    # Certificates paths - Final fixed locations
    # 1. 実際の実行で使われる場所 (install内)
    project_quic_dir = os.path.abspath(os.path.join(bringup_dir, "config", "quic"))
    # 2. scp 用のソースディレクトリ
    src_quic_dir = os.path.abspath(
        os.path.join(bringup_dir, "..", "..", "..", "src", "bringup", "config", "quic")
    )

    os.makedirs(project_quic_dir, exist_ok=True)
    cert = os.path.join(project_quic_dir, "server.crt")
    key = os.path.join(project_quic_dir, "server.key")
    ca_cert = os.path.join(project_quic_dir, "ca.crt")
    ca_key = os.path.join(project_quic_dir, "ca.key")

    quic_available = True
    if not os.path.isfile(cert) or not os.path.isfile(key) or not os.path.isfile(ca_cert):
        try:
            # 1. CA 証明書の生成 (CA:TRUE)
            with tempfile.NamedTemporaryFile(mode="w", suffix=".conf") as f_ca:
                f_ca.write(
                    "[req]\ndistinguished_name=req_distinguished_name\nx509_extensions=v3_ca\nprompt=no\n"
                )
                f_ca.write("[req_distinguished_name]\nCN=roborescue-ca\n")
                f_ca.write(
                    "[v3_ca]\nbasicConstraints=critical,CA:TRUE\nkeyUsage=critical,digitalSignature,keyCertSign\n"
                )
                f_ca.flush()
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
                        ca_key,
                        "-out",
                        ca_cert,
                        "-config",
                        f_ca.name,
                    ],
                    check=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )

            # 2. サーバー証明書の生成 (CA:FALSE, signed by CA)
            with tempfile.NamedTemporaryFile(mode="w", suffix=".conf") as f_srv:
                f_srv.write(
                    "[req]\ndistinguished_name=req_distinguished_name\nreq_extensions=v3_req\nprompt=no\n"
                )
                f_srv.write("[req_distinguished_name]\nCN=roborescue-server\n")
                f_srv.write(
                    "[v3_req]\nbasicConstraints=CA:FALSE\nkeyUsage=critical,digitalSignature,keyEncipherment\n"
                    "extendedKeyUsage=serverAuth,clientAuth\n"
                    "subjectAltName=DNS:localhost,IP:127.0.0.1,IP:10.42.0.1,IP:100.114.200.30\n"
                )
                f_srv.flush()

                # CSR 生成
                csr = cert + ".csr"
                subprocess.run(
                    [
                        "openssl",
                        "req",
                        "-new",
                        "-keyout",
                        key,
                        "-out",
                        csr,
                        "-nodes",
                        "-config",
                        f_srv.name,
                    ],
                    check=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )

                # CA で署名 (有効期間開始を1日前にずらす)
                subprocess.run(
                    [
                        "openssl",
                        "x509",
                        "-req",
                        "-in",
                        csr,
                        "-CA",
                        ca_cert,
                        "-CAkey",
                        ca_key,
                        "-CAcreateserial",
                        "-out",
                        cert,
                        "-days",
                        "3650",
                        "-sha256",
                        "-extfile",
                        f_srv.name,
                        "-extensions",
                        "v3_req",
                    ],
                    check=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                os.remove(csr)

            actions.append(
                LogInfo(msg="[network.launch] generated high-compatibility CA and Server certs")
            )

            # ソースディレクトリへコピー
            # 互換性のため、operator 用の server.crt はロボットの ca.crt をコピーして使う
            # colcon symlink-install 環境では install/ → src/ がシンボリックリンクになるため
            # 同一ファイルチェックが必要
            if os.path.isdir(os.path.dirname(src_quic_dir)):
                os.makedirs(src_quic_dir, exist_ok=True)
                dst_crt = os.path.join(src_quic_dir, "server.crt")
                dst_key = os.path.join(src_quic_dir, "server.key")
                # server.crt が symlink で cert と同一物理ファイルを指す場合はコピーしない
                # (symlink-install 環境で server.crt を ca.crt で上書きするのを防ぐ)
                try:
                    cert_is_dst = os.path.isfile(dst_crt) and os.path.samefile(cert, dst_crt)
                except (FileNotFoundError, OSError):
                    cert_is_dst = False
                if not cert_is_dst:
                    if not os.path.isfile(dst_crt) or not os.path.samefile(ca_cert, dst_crt):
                        shutil.copy2(ca_cert, dst_crt)
                try:
                    key_is_dst = os.path.isfile(dst_key) and os.path.samefile(key, dst_key)
                except (FileNotFoundError, OSError):
                    key_is_dst = False
                if not key_is_dst:
                    if not os.path.isfile(dst_key) or not os.path.samefile(key, dst_key):
                        shutil.copy2(key, dst_key)
                actions.append(
                    LogInfo(msg=f"[network.launch] synced CA cert to {src_quic_dir} for operator")
                )

            # operator_ws へのコピー (ローカル実行用)
            operator_quic_dir = os.path.abspath(
                os.path.join(bringup_dir, "..", "..", "..", "..", "operator_ws", "quic")
            )
            if os.path.isdir(os.path.dirname(operator_quic_dir)):
                os.makedirs(operator_quic_dir, exist_ok=True)
                dst_ope_crt = os.path.join(operator_quic_dir, "server.crt")
                if not os.path.isfile(dst_ope_crt) or not os.path.samefile(ca_cert, dst_ope_crt):
                    shutil.copy2(ca_cert, dst_ope_crt)
                actions.append(
                    LogInfo(msg=f"[network.launch] synced CA cert to {operator_quic_dir}")
                )
        except Exception as e:
            quic_available = False
            actions.append(LogInfo(msg=f"[network.launch] failed to prepare QUIC TLS assets: {e}"))

    # Router config (zenohd用)
    fd, tcp_only_cfg = tempfile.mkstemp(prefix="zenoh_router_tcp_fallback_", suffix=".json5")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(
            '{\n  mode: "router",\n  listen: { endpoints: ["tcp/0.0.0.0:7447"] },\n  scouting: { multicast: { enabled: false } },\n  transport: { shared_memory: { enabled: true } }\n}\n'
        )

    if quic_available:
        fd2, quic_cfg = tempfile.mkstemp(prefix="zenoh_router_quic_", suffix=".json5")
        with os.fdopen(fd2, "w", encoding="utf-8") as f:
            f.write(
                '{\n  mode: "router",\n  listen: { endpoints: ["quic/0.0.0.0:7447", "tcp/0.0.0.0:7447"] },\n  scouting: { multicast: { enabled: false } },\n  transport: {\n    shared_memory: { enabled: true },\n    link: {\n      tls: {\n'
                + f'        listen_private_key: "{key}",\n        listen_certificate: "{cert}"\n'
                + "      }\n    }\n  }\n}\n"
            )
        actions.append(
            LogInfo(
                msg=f"[network.launch] QUIC cert for operator is ready at {src_quic_dir}/server.crt"
            )
        )
        router_config_for_launch = quic_cfg
    else:
        router_config_for_launch = tcp_only_cfg

    # 接続イベントを表示するために RUST_LOG を設定 (info で十分)
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
            # SetEnvironmentVariable("LD_LIBRARY_PATH", patched_ld_library_path),
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
