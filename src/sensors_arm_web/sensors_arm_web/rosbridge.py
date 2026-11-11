"""Minimal async rosbridge WebSocket client (service call + topic subscribe)."""

from __future__ import annotations

import asyncio
import json
import itertools
from typing import Any, Callable, Dict, Optional

import websockets


class RosbridgeClient:
    def __init__(self, url: str = "ws://127.0.0.1:9090") -> None:
        self.url = url
        self._ws: Optional[websockets.WebSocketClientProtocol] = None
        self._id = itertools.count(1)
        self._pending: Dict[str, asyncio.Future] = {}
        self._subs: Dict[str, Callable[[dict], None]] = {}
        self._reader_task: Optional[asyncio.Task] = None
        self._lock = asyncio.Lock()

    @property
    def connected(self) -> bool:
        if self._ws is None:
            return False
        # websockets v10+: .open; v13+: check state name
        open_attr = getattr(self._ws, "open", None)
        if open_attr is not None:
            return bool(open_attr)
        state = getattr(self._ws, "state", None)
        return str(state).endswith("OPEN") if state is not None else True

    async def connect(self, timeout: float = 5.0) -> None:
        async with self._lock:
            if self.connected:
                return
            self._ws = await asyncio.wait_for(websockets.connect(self.url), timeout=timeout)
            self._reader_task = asyncio.create_task(self._read_loop())

    async def close(self) -> None:
        for fut in list(self._pending.values()):
            if not fut.done():
                fut.cancel()
        self._pending.clear()
        if self._reader_task:
            self._reader_task.cancel()
            try:
                await self._reader_task
            except (asyncio.CancelledError, Exception):
                pass
            self._reader_task = None
        if self._ws is not None:
            try:
                await self._ws.close()
            except Exception:
                pass
            self._ws = None

    async def _read_loop(self) -> None:
        assert self._ws is not None
        try:
            async for raw in self._ws:
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                op = msg.get("op")
                if op == "service_response":
                    mid = str(msg.get("id") or "")
                    fut = self._pending.pop(mid, None)
                    if fut and not fut.done():
                        fut.set_result(msg)
                elif op == "publish":
                    topic = msg.get("topic")
                    cb = self._subs.get(topic)
                    if cb:
                        try:
                            cb(msg.get("msg") or {})
                        except Exception:
                            pass
        except Exception:
            # Drop connection; callers reconnect on demand.
            self._ws = None

    async def _send(self, payload: dict) -> None:
        if not self.connected:
            await self.connect()
        assert self._ws is not None
        await self._ws.send(json.dumps(payload))

    async def call_service(
        self,
        service: str,
        srv_type: str,
        args: Optional[dict] = None,
        timeout: float = 30.0,
    ) -> dict:
        mid = str(next(self._id))
        fut: asyncio.Future = asyncio.get_running_loop().create_future()
        self._pending[mid] = fut
        await self._send({
            "op": "call_service",
            "id": mid,
            "service": service,
            "type": srv_type,
            "args": args or {},
        })
        try:
            resp = await asyncio.wait_for(fut, timeout=timeout)
        except Exception:
            self._pending.pop(mid, None)
            raise
        if not resp.get("result", True) and resp.get("values") is None:
            raise RuntimeError(resp.get("values") or "service call failed")
        return resp.get("values") or {}

    async def subscribe(self, topic: str, msg_type: str, callback: Callable[[dict], None]) -> None:
        self._subs[topic] = callback
        await self._send({
            "op": "subscribe",
            "topic": topic,
            "type": msg_type,
        })
