from textual.widgets import Static
from textual.reactive import reactive
from textual.containers import Container


class LinkContainer(Container):
    DEFAULT_CSS = """
    LinkContainer {
        overflow: auto;      /* scroll if needed */
        height: auto;
        content-align: left top;
    }
    """


class Link(Static):
    """A focusable, keyboard-activatable inline 'link' widget."""

    can_focus = True
    action_name = reactive("")
    action_arg = reactive("")

    DEFAULT_CSS = """
    Link {
        display: inline;       /* important for inline wrapping */
        width: auto;
        text-style: underline;
        content-align: left middle;
        color: $accent;
    }
    Link:focus {
        text-style: underline bold;
        background: $boost;
    }
    Link.-type {
        color: $text-muted;    /* example variant for [type] links */
    }
    """

    def __init__(
        self, label: str, action_name: str, action_arg: str, classes: str = ""
    ):
        super().__init__(label, classes=classes)
        self.action_name = action_name
        self.action_arg = action_arg

    def _activate(self) -> None:
        # call an App action if it exists; otherwise fall back to a method
        if self.app and self.action_name:
            # Prefer Textual actions so you can bind keys if you like:
            # e.g. define "action_open_node" on the App and pass "open_node"
            action = f"app.{self.action_name}('{self.action_arg}')"
            self.app.run_action(action)
        else:
            # no-op or custom fallback
            pass

    def on_click(self, _) -> None:
        self._activate()

    def on_key(self, event) -> None:
        if event.key in ("enter", "space"):
            event.stop()
            self._activate()
