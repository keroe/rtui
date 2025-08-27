from typing import Optional

from textual.app import ComposeResult
from textual.containers import Vertical, VerticalGroup
from textual.widgets import TabbedContent, TabPane, Static, Log

from rtui_app.ros import RosClient, RosEntity, ROSBg
from rtui_app.ros.nodes import EchoNode, HzNode


class RosEntityMonitorPanel(Vertical):
    def __init__(self, ros: RosClient, entity: RosEntity | None = None) -> None:
        super().__init__()
        self._ros = ros
        self._entity = entity
        self.hz_monitor = HzMonitor(ros, entity)
        self.bw_monitor = Static("Bandwidth Monitor")
        self.echo_monitor = EchoLog(ros, entity)

    def compose(self) -> ComposeResult:
        with TabbedContent(initial="hz"):
            with TabPane("hz", id="hz"):
                yield self.hz_monitor
            with TabPane("bw", id="bw"):
                yield self.bw_monitor
            with TabPane("echo", id="echo"):
                yield self.echo_monitor

    def set_entity(self, entity: RosEntity) -> None:
        if entity != self._entity:
            self._entity = entity
            self.update_content()

    def update_content(self):
        self.hz_monitor.set_entity(self._entity)
        self.bw_monitor.update(f"Not implemented")
        self.echo_monitor.set_entity(self._entity)


class EchoLog(Vertical):
    CSS = "Log { border: round $accent; height: 100%; }"

    def __init__(self, ros: RosClient, entity: RosEntity | None = None) -> None:
        super().__init__()
        self._ros = ros
        self._entity = entity
        self.echo_ros_node = None

    def compose(self) -> ComposeResult:
        yield Log(highlight=True, auto_scroll=True, max_lines=1000)

    def on_mount(self):
        self.set_interval(0.1, self.push_line)

    def push_line(self):
        if not self.echo_ros_node:
            return
        line = self.echo_ros_node.get_node_status()
        log = self.query_one(Log)
        log.write_line(line)

    def set_entity(self, entity: RosEntity) -> None:
        if entity != self._entity:
            self._entity = entity
            self.update_content()

    def update_content(self):
        self._topic = self._entity.name if self._entity else "N/A"
        if self._topic:
            log = self.query_one(Log)
            log.clear()
            log.write_line(f"Starting echo for {self._topic}...")
            self._start_ros()

    def on_unmount(self) -> None:
        if self.echo_ros_node:
            self.echo_ros_node.stop()
            self.echo_ros_node = None

    def _start_ros(self) -> None:
        self.echo_ros_node = ROSBg(
            self._topic, node_class=EchoNode, executor=self._ros.interface.executor
        )
        self.echo_ros_node.start()


class HzMonitor(VerticalGroup):
    def __init__(self, ros: RosClient, entity: RosEntity | None = None) -> None:
        super().__init__()
        self._ros = ros
        self._entity = entity
        self.hz_ros_node = None

    def compose(self) -> ComposeResult:
        self.hz_display = Static("Starting...")
        yield self.hz_display

    def on_mount(self):
        self.set_interval(1.0, self.update_stats)

    def update_stats(self):
        if not self.hz_ros_node:
            return
        status = self.hz_ros_node.get_node_status()
        self.hz_display.update(status)

    def set_entity(self, entity: RosEntity) -> None:
        if entity != self._entity:
            self._entity = entity
            self.update_content()

    def update_content(self):
        self._topic = self._entity.name if self._entity else "N/A"
        if self._topic:
            self.hz_display.update(f"Starting monitor for {self._topic}...")
            self._start_ros()

    def on_unmount(self) -> None:
        if self.hz_ros_node:
            self.hz_ros_node.stop()
            self.hz_ros_node = None

    def _start_ros(self) -> None:
        self.hz_ros_node = ROSBg(
            self._topic, node_class=HzNode, executor=self._ros.interface.executor
        )
        self.hz_ros_node.start()
