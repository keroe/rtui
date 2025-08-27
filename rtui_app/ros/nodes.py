from rosidl_runtime_py import message_to_yaml
from rclpy.node import Node
from ros2topic.verb.hz import ROSTopicHz
from rosidl_runtime_py.utilities import get_message

import queue


class EchoNode(Node):
    def __init__(self, topic: str):
        super().__init__("auto_textual_echo")
        self._topic = topic
        self._sub = None
        self.lines = queue.Queue(maxsize=1000)
        self.create_timer(0.5, self._try_subscribe)

    def _try_subscribe(self):
        try:
            names_types = dict(self.get_topic_names_and_types())
        except Exception as e:
            self.lines.put(f"[red]Graph query failed:[/red] {e!r}")
            return

        if self._topic not in names_types:
            return  # keep polling

        type_str = names_types[self._topic][0]
        try:
            msg_type = get_message(type_str)
        except Exception as e:
            self.lines.put(f"[red]Failed to import {type_str}:[/red] {e!r}")
            return

        if self._sub is None:
            self._sub = self.create_subscription(msg_type, self._topic, self._cb, 10)
            self.lines.put(
                f"✅ Subscribed to [b]{self._topic}[/b] ([green]{type_str}[/green])"
            )

    def _cb(self, msg):
        # Pretty print as YAML (same style as ros2 topic echo)
        try:
            text = message_to_yaml(msg).rstrip()
        except Exception:
            # fallback if YAML fails
            text = repr(msg)
        self.lines.put(text)


class HzNode(Node):
    """Node that resolves type lazily, then subscribes and emits Hz status."""

    WINDOW_SIZE = 200
    TYPE_POLL_PERIOD = 0.5  # seconds
    HZ_UPDATE_PERIOD = 1.0  # seconds
    TYPE_POLL_TIMEOUT = 15.0  # seconds (set 0/None to wait forever)

    def __init__(self, topic: str):
        super().__init__("auto_hz_textual")
        self._topic = topic
        self._hz = ROSTopicHz(self, window_size=self.WINDOW_SIZE)
        self._sub = None

        # 1) Start polling for the topic type
        self._start_time = self.get_clock().now().nanoseconds / 1e9
        self._type_timer = self.create_timer(
            self.TYPE_POLL_PERIOD, self._try_setup_subscription
        )

        # 2) Timer to push Hz status (works once subscription exists)
        self._hz_timer = self.create_timer(self.HZ_UPDATE_PERIOD, self._emit_status)
        self.status = f"📡 [b]{self._topic}[/b]\n[yellow]⏳ Waiting for messages…[/yellow]\n[dim]🪟 window:[/dim] {self.WINDOW_SIZE:5d}"

    def _push_status(self, msg: str):
        self.status = msg

    def _try_setup_subscription(self):
        # Optional timeout
        if self.TYPE_POLL_TIMEOUT:
            now = self.get_clock().now().nanoseconds / 1e9
            if (now - self._start_time) > self.TYPE_POLL_TIMEOUT:
                self._push_status(
                    f"[red]Timeout[/red]: topic '{self._topic}' not found."
                )
                self.destroy_timer(self._type_timer)
                return

        try:
            types = dict(self.get_topic_names_and_types())
        except Exception as e:
            self._push_status(f"[red]Graph query failed[/red]: {e!r}")
            return

        if self._topic not in types:
            # keep polling
            return

        type_str = types[self._topic][0]
        try:
            msg_type = get_message(type_str)
        except Exception as e:
            self._push_status(f"[red]Failed to import type[/red] {type_str}: {e!r}")
            return

        # Create subscription only once
        if self._sub is None:
            self._sub = self.create_subscription(
                msg_type, self._topic, self._hz.callback_hz, 10
            )
            self._push_status(
                f"📡 [b]{self._topic}[/b]\n[green]✅ Subscribed[/green] ({type_str}). Waiting for messages…"
            )
            self.destroy_timer(self._type_timer)  # stop polling

    def _emit_status(self):
        res = self._hz.get_hz()
        if res:
            hz, mn, mx, std, win = res
            status = (
                f"📡 [b]{self._topic}[/b]\n"
                f"⚡ Rate: {" ":>7}[b]{hz*1e9:.6f} Hz[/b]\n"
                f"🔽 min Δt: {" ":>5}{mn/1e9:.6f} s\n"
                f"🔼 max Δt: {" ":>5}{mx/1e9:.6f} s\n"
                f"📊 std Δt: {" ":>5}{std/1e9:.6f} s\n"
                f"🪟 window: {" ":>5}{win}"
            )
            self._push_status(status)


class ROSBg:
    def __init__(self, topic: str, *, node_class: HzNode | EchoNode, executor=None):
        self.topic = topic
        self.exec = executor
        self.node = None
        self.node_class = node_class

    def start(self):
        # Create node and add to shared executor
        if self.node is not None:
            return  # already started
        self.node = self.node_class(self.topic)
        if self.exec is not None:
            self.exec.add_node(self.node)

    def stop(self):
        # Remove node from executor and destroy
        if self.exec and self.node:
            self.exec.remove_node(self.node)
        if self.node:
            self.node.destroy_node()
        self.node = None

    def get_node_status(self) -> str:
        if isinstance(self.node, HzNode):
            return self.node.status
        if isinstance(self.node, EchoNode):
            return self.node.lines.get()
        return "Not started"
