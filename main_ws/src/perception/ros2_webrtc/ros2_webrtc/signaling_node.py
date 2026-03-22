#!/usr/bin/env python3
"""
ROS2 signaling node: runs a WebSocket signaling server and bridges messages
to local ROS2 topics so other ROS2 nodes can publish/subscribe signaling
messages. This allows ROS2-managed signaling while still letting remote peers
connect via WebSocket.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import asyncio
import json
from aiohttp import web, WSMsgType

rooms = {}  # room -> set of websocket


class SignalingNode(Node):
    def __init__(self):
        super().__init__("ros2_signaling")
        self.pub_incoming = self.create_publisher(String, "webrtc/incoming", 10)
        self.sub_outgoing = self.create_subscription(
            String, "webrtc/outgoing", self.on_outgoing, 10
        )

    def on_outgoing(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        room = data.get("room", "default")
        # Broadcast to websockets in the room
        for ws in list(rooms.get(room, set())):
            asyncio.ensure_future(ws.send_str(json.dumps(data)))


async def websocket_handler(request):
    import os
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    import asyncio
    import json
    import ssl
    from aiohttp import web, WSMsgType

    rooms = {}  # room -> set of websocket

    class SignalingNode(Node):
        def __init__(self):
            super().__init__("ros2_signaling")
            self.pub_incoming = self.create_publisher(String, "webrtc/incoming", 10)
            self.sub_outgoing = self.create_subscription(
                String, "webrtc/outgoing", self.on_outgoing, 10
            )
            self.auth_token = os.environ.get("SIGNALING_AUTH_TOKEN")
            # optional ICE servers JSON string in env: [{'urls': 'stun:...'},{'urls':'turn:...', 'username':'u','credential':'p'}]
            ice_env = os.environ.get("SIGNALING_ICE_SERVERS")
            try:
                self.ice_servers = json.loads(ice_env) if ice_env else None
            except Exception:
                self.ice_servers = None

        def on_outgoing(self, msg):
            try:
                data = json.loads(msg.data)
            except Exception:
                return
            room = data.get("room", "default")
            # Broadcast to websockets in the room
            for ws in list(rooms.get(room, set())):
                try:
                    asyncio.ensure_future(ws.send_str(json.dumps(data)))
                except Exception:
                    pass

    async def websocket_handler(request):
        # Authorization check (Bearer token) if configured
        node = request.app["node"]
        auth_ok = True
        if node.auth_token:
            auth_header = request.headers.get("Authorization", "")
            if auth_header.startswith("Bearer "):
                token = auth_header.split(" ", 1)[1]
                auth_ok = token == node.auth_token
            else:
                # allow token via query param for legacy clients
                token = request.query.get("token")
                auth_ok = token == node.auth_token

        if not auth_ok:
            return web.Response(status=401, text="unauthorized")

        ws = web.WebSocketResponse()
        await ws.prepare(request)

        room = None
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                    except Exception:
                        await ws.send_str(json.dumps({"error": "invalid json"}))
                        continue

                    if data.get("type") == "join":
                        room = data.get("room") or "default"
                        rooms.setdefault(room, set()).add(ws)
                        reply = {"type": "joined", "room": room}
                        if node.ice_servers:
                            reply["iceServers"] = node.ice_servers
                        await ws.send_str(json.dumps(reply))
                        continue

                    # publish incoming to ROS topic
                    node.get_logger().debug("signaling incoming: %s" % str(data))
                    node.pub_incoming.publish(String(data=json.dumps(data)))

                elif msg.type == WSMsgType.ERROR:
                    break
        finally:
            if room and ws in rooms.get(room, set()):
                rooms[room].remove(ws)
                if not rooms[room]:
                    del rooms[room]

        return ws

    def build_ssl_context():
        cert = os.environ.get("SIGNALING_TLS_CERT")
        key = os.environ.get("SIGNALING_TLS_KEY")
        if not cert or not key:
            return None
        ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ctx.load_cert_chain(certfile=cert, keyfile=key)
        return ctx

    def main():
        rclpy.init()
        node = SignalingNode()

        app = web.Application()
        app["node"] = node
        app.router.add_get("/ws", websocket_handler)

        loop = asyncio.get_event_loop()
        runner = web.AppRunner(app)

        ssl_ctx = build_ssl_context()

        async def start():
            await runner.setup()
            host = os.environ.get("SIGNALING_HOST", "0.0.0.0")
            port = int(os.environ.get("SIGNALING_PORT", "8080"))
            site = web.TCPSite(runner, host, port, ssl_context=ssl_ctx)
            await site.start()
            node.get_logger().info(
                f"Signaling server started on {host}:{port} (wss={bool(ssl_ctx)})"
            )

        loop.run_until_complete(start())

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            loop.run_until_complete(runner.cleanup())
            node.destroy_node()
            rclpy.shutdown()

    if __name__ == "__main__":
        main()
