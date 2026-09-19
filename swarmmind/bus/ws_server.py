"""Minimal WebSocket server, stdlib only.

Godot 4 talks WebSocket natively (`WebSocketPeer`), and the `godot_ros` GDExtension is
not an option here: it needs ROS2 headers on the same host as the editor, and ROS2 would
be inside Docker while Godot runs natively on macOS. So this is the *primary* transport
to the dashboard, not a fallback (docs/PLAN.md D4).

Deliberately not the `websockets` package: this is one handshake plus RFC 6455 frame
headers, it needs no install, and it keeps the 8 GB budget untouched. Everything the
dashboard needs is server -> client text frames; client -> server is a handful of
one-line commands.
"""

from __future__ import annotations

import base64
import contextlib
import hashlib
import json
import socket
import struct
import threading

_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

OP_TEXT = 0x1
OP_CLOSE = 0x8
OP_PING = 0x9
OP_PONG = 0xA


def _encode(payload: bytes, opcode: int = OP_TEXT) -> bytes:
    """One unmasked server frame. Server frames are never masked (RFC 6455 5.1)."""
    n = len(payload)
    head = bytes([0x80 | opcode])
    if n < 126:
        head += bytes([n])
    elif n < (1 << 16):
        head += bytes([126]) + struct.pack(">H", n)
    else:
        head += bytes([127]) + struct.pack(">Q", n)
    return head + payload


class _Client:
    def __init__(self, conn: socket.socket, addr) -> None:
        self.conn = conn
        self.addr = addr
        self.alive = True
        self.buf = b""

    def send(self, frame: bytes) -> bool:
        try:
            self.conn.sendall(frame)
            return True
        except OSError:
            self.alive = False
            return False

    def close(self) -> None:
        self.alive = False
        with contextlib.suppress(OSError):
            self.conn.close()


class WebSocketServer:
    """Threaded broadcast server. One accept thread, one reader thread per client."""

    def __init__(self, host: str = "127.0.0.1", port: int = 8765) -> None:
        self.host, self.port = host, port
        self._sock: socket.socket | None = None
        self._clients: list[_Client] = []
        self._lock = threading.Lock()
        self._commands: list[dict] = []
        self._running = False
        self.on_connect = None      # called with the client; used to send `hello`
        self.connections = 0

    # ------------------------------------------------------------------ lifecycle

    def start(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, self.port))
        s.listen(8)
        s.settimeout(0.5)
        self._sock = s
        self._running = True
        threading.Thread(target=self._accept_loop, daemon=True, name="ws-accept").start()

    def stop(self) -> None:
        self._running = False
        with self._lock:
            for c in self._clients:
                c.close()
            self._clients.clear()
        if self._sock:
            with contextlib.suppress(OSError):
                self._sock.close()

    @property
    def client_count(self) -> int:
        with self._lock:
            return sum(1 for c in self._clients if c.alive)

    # ------------------------------------------------------------------ sending

    def broadcast(self, obj) -> None:
        """Send one JSON object to every connected dashboard."""
        frame = _encode(json.dumps(obj, separators=(",", ":")).encode())
        with self._lock:
            dead = []
            for c in self._clients:
                if not c.send(frame):
                    dead.append(c)
            for c in dead:
                self._clients.remove(c)

    def send_to(self, client: _Client, obj) -> None:
        client.send(_encode(json.dumps(obj, separators=(",", ":")).encode()))

    def poll_commands(self) -> list[dict]:
        with self._lock:
            out, self._commands = self._commands, []
        return out

    # ------------------------------------------------------------------ internals

    def _accept_loop(self) -> None:
        while self._running:
            try:
                conn, addr = self._sock.accept()
            except (TimeoutError, OSError):
                continue
            try:
                if not self._handshake(conn):
                    conn.close()
                    continue
            except OSError:
                conn.close()
                continue
            client = _Client(conn, addr)
            with self._lock:
                self._clients.append(client)
                self.connections += 1
            if self.on_connect:
                self.on_connect(client)
            threading.Thread(target=self._read_loop, args=(client,),
                             daemon=True, name="ws-read").start()

    def _handshake(self, conn: socket.socket) -> bool:
        conn.settimeout(5.0)
        data = b""
        while b"\r\n\r\n" not in data:
            chunk = conn.recv(4096)
            if not chunk:
                return False
            data += chunk
            if len(data) > 16384:
                return False
        headers = {}
        for line in data.split(b"\r\n")[1:]:
            if b":" in line:
                k, _, v = line.partition(b":")
                headers[k.strip().lower()] = v.strip()
        key = headers.get(b"sec-websocket-key")
        if not key:
            return False
        accept = base64.b64encode(hashlib.sha1(key + _GUID).digest()).decode()
        conn.sendall(
            b"HTTP/1.1 101 Switching Protocols\r\n"
            b"Upgrade: websocket\r\n"
            b"Connection: Upgrade\r\n"
            b"Sec-WebSocket-Accept: " + accept.encode() + b"\r\n\r\n"
        )
        conn.settimeout(None)
        return True

    def _read_loop(self, client: _Client) -> None:
        while self._running and client.alive:
            try:
                frame = self._read_frame(client)
            except (OSError, ValueError):
                break
            if frame is None:
                break
            opcode, payload = frame
            if opcode == OP_CLOSE:
                break
            if opcode == OP_PING:
                client.send(_encode(payload, OP_PONG))
                continue
            if opcode == OP_TEXT:
                try:
                    msg = json.loads(payload.decode())
                except (ValueError, UnicodeDecodeError):
                    continue
                with self._lock:
                    self._commands.append(msg)
        client.close()
        with self._lock:
            if client in self._clients:
                self._clients.remove(client)

    def _read_frame(self, client: _Client):
        b = self._recv_exact(client, 2)
        if b is None:
            return None
        opcode = b[0] & 0x0F
        masked = bool(b[1] & 0x80)
        n = b[1] & 0x7F
        if n == 126:
            ext = self._recv_exact(client, 2)
            if ext is None:
                return None
            n = struct.unpack(">H", ext)[0]
        elif n == 127:
            ext = self._recv_exact(client, 8)
            if ext is None:
                return None
            n = struct.unpack(">Q", ext)[0]
        if n > (1 << 22):
            raise ValueError("client frame too large")
        mask = self._recv_exact(client, 4) if masked else b"\x00\x00\x00\x00"
        if mask is None:
            return None
        payload = self._recv_exact(client, n) if n else b""
        if payload is None:
            return None
        if masked:
            payload = bytes(byte ^ mask[i % 4] for i, byte in enumerate(payload))
        return opcode, payload

    @staticmethod
    def _recv_exact(client: _Client, n: int) -> bytes | None:
        out = b""
        while len(out) < n:
            chunk = client.conn.recv(n - len(out))
            if not chunk:
                return None
            out += chunk
        return out
