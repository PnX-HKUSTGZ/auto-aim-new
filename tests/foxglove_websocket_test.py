#!/usr/bin/env python3
"""Foxglove protocol smoke test, using only the Python standard library."""

import base64
import hashlib
import json
import os
import socket
import struct
import subprocess
import sys
import time


def read_exact(sock, count):
    data = b""
    while len(data) < count:
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise RuntimeError("Unexpected end of WebSocket stream")
        data += chunk
    return data


def connect(port):
    sock = socket.create_connection(("127.0.0.1", port), timeout=2)
    key = base64.b64encode(os.urandom(16)).decode()
    sock.sendall((
        f"GET / HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n"
        "Upgrade: websocket\r\nConnection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n"
        "Sec-WebSocket-Protocol: foxglove.websocket.v1\r\n\r\n"
    ).encode())
    response = b""
    while not response.endswith(b"\r\n\r\n"):
        response += read_exact(sock, 1)
    expected = base64.b64encode(hashlib.sha1(
        (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest())
    assert response.startswith(b"HTTP/1.1 101 "), response
    assert expected in response
    assert b"sec-websocket-protocol: foxglove.websocket.v1" in response.lower()
    return sock


def send(sock, message):
    payload = json.dumps(message).encode()
    mask = os.urandom(4)
    header = bytes([0x81, 0x80 | len(payload)]) if len(payload) < 126 else (
        bytes([0x81, 0x80 | 126]) + struct.pack("!H", len(payload)))
    sock.sendall(header + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))


def receive(sock):
    payload = b""
    first_opcode = None
    while True:
        first, second = read_exact(sock, 2)
        opcode = first & 0x0F
        assert not second & 0x80, "Server frames must not be masked"
        length = second & 0x7F
        if length == 126:
            length = struct.unpack("!H", read_exact(sock, 2))[0]
        elif length == 127:
            length = struct.unpack("!Q", read_exact(sock, 8))[0]
        assert length < 1024 * 1024
        payload += read_exact(sock, length)
        if first_opcode is None:
            first_opcode = opcode
        else:
            assert opcode == 0, "Expected continuation frame"
        if first & 0x80:
            return first_opcode, payload


def validate(value, schema):
    """Validate the types and constraints used by the vendored Foxglove schema."""
    kind = schema.get("type")
    if kind:
        types = {"object": dict, "array": list, "string": str, "boolean": bool,
                 "integer": int, "number": (int, float)}
        assert isinstance(value, types[kind]), (value, kind)
        if kind in ("integer", "number"):
            assert not isinstance(value, bool)
    if "const" in schema:
        assert value == schema["const"]
    if "oneOf" in schema:
        matches = 0
        for option in schema["oneOf"]:
            try:
                validate(value, option)
                matches += 1
            except AssertionError:
                pass
        assert matches == 1
    if isinstance(value, dict):
        assert all(key in value for key in schema.get("required", []))
        for key, field in schema.get("properties", {}).items():
            if key in value:
                validate(value[key], field)
    elif isinstance(value, list):
        for item in value:
            validate(item, schema.get("items", {}))
    elif isinstance(value, (int, float)):
        assert value >= schema.get("minimum", float("-inf"))
        assert value <= schema.get("maximum", float("inf"))


def advertised(sock):
    opcode, payload = receive(sock)
    assert opcode == 1 and json.loads(payload)["op"] == "serverInfo"
    opcode, payload = receive(sock)
    message = json.loads(payload)
    assert opcode == 1 and message["op"] == "advertise"
    channels = {channel["id"]: channel for channel in message["channels"]}
    scene_id = next(key for key, value in channels.items()
                    if value["schemaName"] == "foxglove.SceneUpdate")
    transform_id = next(key for key, value in channels.items()
                        if value["schemaName"] == "foxglove.FrameTransform")
    channel = channels[scene_id]
    assert channel["encoding"] == "json"
    transform = channels[transform_id]
    assert transform["schemaName"] == "foxglove.FrameTransform"
    assert transform["encoding"] == "json"
    schemas = {key: json.loads(value["schema"]) for key, value in channels.items()}
    return schemas, scene_id, transform_id, channel["topic"]


def snapshot(sock, subscription, schema, expected_entity):
    opcode, payload = receive(sock)
    assert opcode == 2 and payload[0] == 1
    sub, timestamp = struct.unpack("<IQ", payload[1:13])
    assert sub == subscription and timestamp == 1700000000123456789
    scene = json.loads(payload[13:])
    validate(scene, schema)
    assert any(e["id"] == expected_entity for e in scene["entities"])
    assert all(e["frame_id"] == "world" for e in scene["entities"])
    return scene


def transform_snapshot(sock, subscription, schema, expected_child):
    opcode, payload = receive(sock)
    assert opcode == 2 and payload[0] == 1
    sub, timestamp = struct.unpack("<IQ", payload[1:13])
    assert sub == subscription and timestamp == 1700000000123456789
    transform = json.loads(payload[13:])
    validate(transform, schema)
    assert transform["parent_frame_id"] == "world"
    assert transform["child_frame_id"] == expected_child
    expected_translation = ({"x": 0.1, "y": -0.2, "z": 0.3}
                            if expected_child == "camera"
                            else {"x": 0.0, "y": 0.0, "z": 0.0})
    assert transform["translation"] == expected_translation
    assert transform["timestamp"] == {"sec": 1700000000, "nsec": 123456789}


def main():
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    process = subprocess.Popen([sys.argv[1], "--serve", str(port)])
    try:
        deadline = time.monotonic() + 3
        while True:
            try:
                sock = connect(port)
                break
            except ConnectionRefusedError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.03)
        with sock:
            schema, scene_id, transform_id, scene_topic = advertised(sock)
            expected_entity = ("rune_plane" if scene_topic == "/buff/scene"
                               else "ballistic_trajectory")
            expected_child = "camera" if scene_topic == "/buff/scene" else "gimbal"
            # A 3D panel must discover frames before it enables any scene topic.
            send(sock, {"op": "subscribe", "subscriptions": [
                {"id": 99, "channelId": transform_id}]})
            transform_snapshot(sock, 99, schema[transform_id], expected_child)
            send(sock, {"op": "unsubscribe", "subscriptionIds": [99]})
            sock.settimeout(0.15)
            while True:
                try:
                    receive(sock)
                except socket.timeout:
                    break
            sock.settimeout(2)
            # Subscription IDs belong to the client, and must not be replaced by channel IDs.
            send(sock, {"op": "subscribe", "subscriptions": [
                {"id": 42, "channelId": scene_id}]})
            snapshot(sock, 42, schema[scene_id], expected_entity)
            send(sock, {"op": "unsubscribe", "subscriptionIds": [42]})
            sock.settimeout(0.15)
            # Drain frames already in flight, then expect silence.
            for _ in range(10):
                try:
                    receive(sock)
                except socket.timeout:
                    break
            else:
                raise AssertionError("Unsubscribe did not stop messages")
        with connect(port) as reconnect:
            schema, scene_id, transform_id, scene_topic = advertised(reconnect)
            expected_entity = ("rune_plane" if scene_topic == "/buff/scene"
                               else "ballistic_trajectory")
            expected_child = "camera" if scene_topic == "/buff/scene" else "gimbal"
            send(reconnect, {"op": "subscribe", "subscriptions": [
                {"id": 73, "channelId": scene_id},
                {"id": 74, "channelId": transform_id}]})
            # Transform is delivered first, regardless of subscription ID ordering.
            transform_snapshot(reconnect, 74, schema[transform_id], expected_child)
            snapshot(reconnect, 73, schema[scene_id], expected_entity)
        assert process.wait(timeout=8) == 0
        print("Foxglove frame discovery, transform-before-scene, schema, unsubscribe and reconnect passed")
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=3)


if __name__ == "__main__":
    main()
