from __future__ import annotations

from textual.app import ComposeResult
from textual.containers import Horizontal, ScrollableContainer, Vertical
from textual.screen import Screen
from textual.widgets import Footer, Static, DataTable

from rtui_app.ros import RosClient, RosEntity, RosEntityType
from rtui_app.widgets import (
    RosEntityInfoPanel,
    RosEntityListPanel,
    RosTypeDefinitionPanel,
    RosEntityMonitorPanel,
)


class RosEntityInspection(Screen):
    _entity_type: RosEntityType
    _entity_name: str | None
    _list_panel: RosEntityListPanel
    _info_panel: RosEntityInfoPanel
    _definition_panel: RosTypeDefinitionPanel | None = None
    _monitor_panel: RosEntityMonitorPanel | None = None

    DEFAULT_CSS = """
    .container {
        height: 100%;
        background: $panel;
    }

    RosEntityListPanel {
        padding-left: 2;
        width: 30%;
    }

    #main {
        border-left: inner $primary;
    }

    #main-upper {
        border-bottom: inner $primary;
    }

    .main-half {
        height: 50%;
    }
    """

    def __init__(self, ros: RosClient, entity_type: RosEntityType) -> None:
        super().__init__()
        self._entity_type = entity_type
        self._entity_name = None
        self._list_panel = RosEntityListPanel(ros, entity_type)
        self._info_panel = RosEntityInfoPanel(
            ros,
            None,
            update_interval=5.0,
        )
        if entity_type.has_definition():
            self._definition_panel = RosTypeDefinitionPanel(ros)
        if entity_type.has_monitor():
            self._monitor_panel = RosEntityMonitorPanel(ros, None)

    def set_entity_name(self, name: str) -> None:
        self._entity_name = name
        entity = RosEntity(type=self._entity_type, name=self._entity_name)
        self._info_panel.set_entity(entity)
        if self._definition_panel is not None:
            self._definition_panel.set_entity(entity)
        if self._monitor_panel is not None:
            self._monitor_panel.set_entity(entity)

    def force_update(self) -> None:
        self._list_panel.update_items()

    def compose(self) -> ComposeResult:
        yield Footer()
        with Horizontal(classes="container"):
            yield self._list_panel

            with Vertical(id="main"):
                if self._definition_panel:
                    with ScrollableContainer(id="main-upper", classes="main-half"):
                        yield self._info_panel
                    with ScrollableContainer(classes="main-half"):
                        yield self._definition_panel
                if self._monitor_panel:
                    with ScrollableContainer(id="main-upper", classes="main-half"):
                        yield self._info_panel
                    with ScrollableContainer(classes="main-half"):
                        yield self._monitor_panel
                else:
                    with ScrollableContainer():
                        yield self._info_panel


class OrphansScreen(Screen):
    CSS = """
Screen {
  layout: vertical;
}

#root {
  padding: 1 2;
}

#root Horizontal {
  height: 1fr;
}

#root DataTable {
  border: round $accent;
}
"""
    BINDINGS = [
        ("i", "go_inspect", "Inspect"),
        ("q", "app.quit", "Quit"),
    ]

    def __init__(self, ros: "Ros2"):
        super().__init__()
        self._ros = ros
        self._timer = None
        self._tbl_no_subs: DataTable | None = None
        self._tbl_no_pubs: DataTable | None = None
        self._status: Static | None = None

    def compose(self) -> ComposeResult:
        with Vertical(id="root"):
            self._status = Static("Finding orphaned topics…")
            yield self._status
            with Horizontal():
                self._tbl_no_subs = DataTable(zebra_stripes=True)
                self._tbl_no_subs.add_columns("Topic", "Types", "Publishers")
                yield self._tbl_no_subs

                self._tbl_no_pubs = DataTable(zebra_stripes=True)
                self._tbl_no_pubs.add_columns("Topic", "Types", "Subscribers")
                yield self._tbl_no_pubs

    def on_mount(self) -> None:
        # refresh every 2s
        self._timer = self.set_interval(2.0, self.refresh_data, pause=False)
        self.refresh_data()

    def on_unmount(self) -> None:
        if self._timer:
            self._timer.stop()
            self._timer = None

    def refresh_data(self) -> None:
        try:
            no_subs, no_pubs = self._ros.interface.find_orphaned_topics(
                include_hidden=True
            )
        except Exception as e:
            self._status.update(f"[red]Graph query failed:[/red] {e!r}")
            return

        self._status.update(
            f"Orphans: no-subs=[b]{len(no_subs)}[/b], no-pubs=[b]{len(no_pubs)}[/b]"
        )

        # refill tables
        self._tbl_no_subs.clear()
        for t in no_subs:
            types = ", ".join(t.types) if t.types else "—"
            pubs = (
                "\n".join(f"{ep.node} ({ep.type or 'unknown'})" for ep in t.publishers)
                or "—"
            )
            self._tbl_no_subs.add_row(t.name, types, pubs)

        self._tbl_no_pubs.clear()
        for t in no_pubs:
            types = ", ".join(t.types) if t.types else "—"
            subs = (
                "\n".join(f"{ep.node} ({ep.type or 'unknown'})" for ep in t.subscribers)
                or "—"
            )
            self._tbl_no_pubs.add_row(t.name, types, subs)

    def action_go_inspect(self) -> None:
        # switch back to your Inspect screen (replace "inspect" with your actual screen id)
        self.app.push_screen("inspect")
