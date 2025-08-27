from typing import Optional

from textual.app import ComposeResult
from textual.containers import Vertical, VerticalGroup
from textual.reactive import reactive
from textual.widgets import TabbedContent, TabPane, Static

from rtui_app.ros import RosClient, RosEntity, ROSBg


class RosEntityMonitorPanel(Vertical):
    def __init__(self, ros: RosClient, entity: RosEntity | None = None) -> None:
        super().__init__()
        self._ros = ros
        self._entity = entity
        self.hz_monitor = HzMonitor(ros, entity)
        self.bw_monitor = Static("Bandwidth Monitor")
        self.echo_monitor = Static("Echo Monitor")

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
        self.echo_monitor.update(f"Echoing data from {self._entity.name}...")


class HzMonitor(VerticalGroup):
    topic: reactive[str | None] = reactive(None)

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
        self.hz_ros_node = ROSBg(self._topic, self._ros.interface.executor)
        self.hz_ros_node.start()
