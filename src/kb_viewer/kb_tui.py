#!/usr/bin/env python3

"""TUI viewer for the KnowledgeCore knowledge base (requires textual)."""

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from datetime import datetime

try:
    import rclpy
    from rclpy.node import Node
    from kb_msgs.srv import Query
except ImportError:
    print("Unable to load ROS 2 support. Make sure to source your ROS 2 workspace.")
    sys.exit(1)

try:
    from textual.app import App, ComposeResult
    from textual.containers import Vertical
    from textual.widgets import DataTable, Footer, Header, Input, Label, Tab, Tabs
except ImportError:
    print("textual is not installed. Run: pip install textual")
    sys.exit(1)

try:
    import pyperclip
except ImportError:
    pyperclip = None


# ── Background ROS fetcher ────────────────────────────────────────────────────

class KBFetcher:
    """
    Runs in a background thread.

    Periodically discovers available KB namespaces (any service ending in
    /kb/query) and queries all triples from the currently active namespace.
    """

    def __init__(self, period: float):
        # Only modified from the loop thread
        self._namespace: str | None = None
        self._period = period
        self._query_client = None

        # Shared state — always access under _lock
        self._triples: list[dict] = []
        self._error: str | None = None
        self._seen_at: dict[tuple, float] = {}
        self._available_namespaces: list[str] = []
        self._pending_namespace: str | None = None
        self._lock = threading.Lock()

        self._running = False
        self._node: Node | None = None

    # ── Public API ────────────────────────────────────────────────────────────

    def start(self) -> None:
        rclpy.init()
        self._node = Node("kb_tui")
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self) -> None:
        self._running = False
        if self._node:
            self._node.destroy_node()
        rclpy.shutdown()

    def get(self) -> tuple[list[dict], dict[tuple, float], str | None]:
        with self._lock:
            return list(self._triples), dict(self._seen_at), self._error

    def get_namespace(self) -> str | None:
        with self._lock:
            return self._pending_namespace if self._pending_namespace else self._namespace

    def get_available_namespaces(self) -> list[str]:
        with self._lock:
            return list(self._available_namespaces)

    def set_namespace(self, ns: str) -> None:
        with self._lock:
            self._pending_namespace = ns.rstrip("/")

    # ── Loop ─────────────────────────────────────────────────────────────────

    def _loop(self) -> None:
        while self._running:
            self._apply_pending_namespace()
            self._discover_namespaces()
            if self._namespace is not None:
                self._fetch()
            time.sleep(self._period)

    def _apply_pending_namespace(self) -> None:
        with self._lock:
            pending = self._pending_namespace
            self._pending_namespace = None

        if pending is None or pending == self._namespace:
            return

        self._namespace = pending
        self._query_client = self._node.create_client(Query, pending + "/query")
        with self._lock:
            self._triples = []
            self._seen_at = {}
            self._error = None

    def _discover_namespaces(self) -> None:
        try:
            services = self._node.get_service_names_and_types()
        except Exception:
            return

        namespaces = [
            name[: -len("/query")]
            for name, types in services
            if name.endswith("/kb/query") and "kb_msgs/srv/Query" in types
        ]
        with self._lock:
            self._available_namespaces = sorted(namespaces)

    def _fetch(self) -> None:
        req = Query.Request()
        req.patterns = ["?s ?p ?o"]
        req.vars = []
        req.models = []

        future = self._query_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)

        result = future.result()
        with self._lock:
            if result is None:
                self._error = "Service call timed out"
            elif not result.success:
                self._error = result.error_msg
            else:
                now = time.time()
                triples = json.loads(result.json)
                current_keys = {(t.get("s", ""), t.get("p", ""), t.get("o", "")) for t in triples}
                for key in current_keys:
                    if key not in self._seen_at:
                        self._seen_at[key] = now
                self._seen_at = {k: v for k, v in self._seen_at.items() if k in current_keys}
                self._triples = triples
                self._error = None


# ── Column definitions ────────────────────────────────────────────────────────

# Each entry: (label, sort_key_fn(triple, seen_at) -> comparable)
_COLUMNS = [
    ("Subject",   lambda t, sa: t.get("s", "").lower()),
    ("Predicate", lambda t, sa: t.get("p", "").lower()),
    ("Object",    lambda t, sa: t.get("o", "").lower()),
    ("Added at",  lambda t, sa: sa.get((t.get("s", ""), t.get("p", ""), t.get("o", "")), 0.0)),
]


def _ns_to_tab_id(ns: str) -> str:
    return "ns_" + ns.strip("/").replace("/", "_").replace("-", "_")


# ── Textual TUI ───────────────────────────────────────────────────────────────

CSS = """
Screen {
    layout: vertical;
}

Tabs {
    height: 3;
}

#filter-bar {
    height: 3;
    padding: 0 1;
    background: $panel;
    border-bottom: solid $primary;
}

#filter-input {
    width: 1fr;
}

#status {
    height: 1;
    padding: 0 1;
    background: $panel-darken-1;
    color: $text-muted;
    text-align: right;
}

DataTable {
    height: 1fr;
}
"""


class KBApp(App):
    """TUI for the KnowledgeCore knowledge base."""

    TITLE = "KnowledgeCore TUI"
    CSS = CSS
    BINDINGS = [
        ("q", "quit", "Quit"),
        ("ctrl+c", "quit", "Quit"),
        ("c", "copy_facts", "Copy facts"),
    ]

    def __init__(self, fetcher: KBFetcher, **kwargs):
        super().__init__(**kwargs)
        self._fetcher = fetcher
        self._sort_col: int = 0
        self._sort_asc: bool = True
        self._col_keys: list = []
        self._tab_to_ns: dict[str, str] = {}
        self._known_tabs: set[str] = set()

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield Tabs(id="ns-tabs")
        with Vertical(id="filter-bar"):
            yield Input(placeholder="Filter by subject, predicate or object…", id="filter-input")
        yield DataTable(id="facts-table")
        yield Label("", id="status")
        yield Footer()

    def on_mount(self) -> None:
        table = self.query_one("#facts-table", DataTable)
        self._col_keys = list(table.add_columns(*[col[0] for col in _COLUMNS]))
        table.cursor_type = "row"

        self.set_interval(1.0, self._refresh_facts)
        self.set_interval(2.0, self._check_namespaces)

    # ── Namespace tab management ──────────────────────────────────────────────

    def _add_tab(self, ns: str) -> None:
        tab_id = _ns_to_tab_id(ns)
        if tab_id in self._known_tabs:
            return
        self.query_one("#ns-tabs", Tabs).add_tab(Tab(ns, id=tab_id))
        self._tab_to_ns[tab_id] = ns
        self._known_tabs.add(tab_id)

    def _check_namespaces(self) -> None:
        available = set(self._fetcher.get_available_namespaces())
        tabs = self.query_one("#ns-tabs", Tabs)
        active_id = tabs.active
        no_tabs_before = len(self._known_tabs) == 0

        for ns in sorted(available):
            self._add_tab(ns)

        # Remove tabs whose namespace is no longer available, except the active one
        for tab_id in list(self._known_tabs):
            ns = self._tab_to_ns[tab_id]
            if ns not in available and tab_id != active_id:
                tabs.remove_tab(tab_id)
                self._known_tabs.discard(tab_id)
                del self._tab_to_ns[tab_id]

        # Auto-activate the first tab when namespaces appear for the first time
        if no_tabs_before and self._known_tabs:
            first_id = _ns_to_tab_id(sorted(available)[0])
            tabs.active = first_id

        ns = self._fetcher.get_namespace()
        self.sub_title = ns if ns else "scanning…"

    def on_tabs_tab_activated(self, event: Tabs.TabActivated) -> None:
        if event.tab is None:
            return
        ns = self._tab_to_ns.get(event.tab.id, "")
        if ns:
            self._fetcher.set_namespace(ns)
            self.sub_title = ns
            # Clear table so stale data from the old namespace isn't shown
            self.query_one("#facts-table", DataTable).clear()

    # ── Sorting ───────────────────────────────────────────────────────────────

    def on_data_table_header_selected(self, event: DataTable.HeaderSelected) -> None:
        col_idx = self._col_keys.index(event.column_key)
        if self._sort_col == col_idx:
            self._sort_asc = not self._sort_asc
        else:
            self._sort_col = col_idx
            self._sort_asc = True
        self._refresh_facts()

    # ── Facts refresh ─────────────────────────────────────────────────────────

    def on_input_changed(self, event: Input.Changed) -> None:
        self._refresh_facts()

    def _refresh_facts(self) -> None:
        triples, seen_at, error = self._fetcher.get()
        filt = self.query_one("#filter-input", Input).value.strip().lower()
        status_label = self.query_one("#status", Label)
        table = self.query_one("#facts-table", DataTable)

        if error:
            status_label.update(f"[red]Error: {error}[/red]")
            return

        visible = [
            t for t in triples
            if not filt
            or filt in t.get("s", "").lower()
            or filt in t.get("p", "").lower()
            or filt in t.get("o", "").lower()
        ]

        sort_fn = _COLUMNS[self._sort_col][1]
        visible.sort(key=lambda t: sort_fn(t, seen_at), reverse=not self._sort_asc)

        def fmt_ts(t: dict) -> str:
            key = (t.get("s", ""), t.get("p", ""), t.get("o", ""))
            ts = seen_at.get(key)
            return datetime.fromtimestamp(ts).strftime("%H:%M:%S") if ts else ""

        new_rows = [
            (t.get("s", ""), t.get("p", ""), t.get("o", ""), fmt_ts(t))
            for t in visible
        ]
        old_rows = [table.get_row_at(i) for i in range(table.row_count)]

        if new_rows != old_rows:
            scroll_y = table.scroll_y
            table.clear()
            for row in new_rows:
                table.add_row(*row)
            table.scroll_to(y=scroll_y, animate=False)

        total = len(triples)
        shown = len(visible)
        sort_indicator = f"{'▲' if self._sort_asc else '▼'} {_COLUMNS[self._sort_col][0]}"
        base = f"Showing {shown} of {total} facts" if filt else f"{total} facts"
        filt_part = f"  |  filter: '{filt}'" if filt else ""
        status_label.update(f"{base}{filt_part}  |  sort: {sort_indicator}  |  ns: {self._fetcher.get_namespace()}")

    # ── Copy facts ────────────────────────────────────────────────────────────

    def action_copy_facts(self) -> None:
        """Copy all visible facts in the active tab to the clipboard."""
        table = self.query_one("#facts-table", DataTable)
        status_label = self.query_one("#status", Label)

        if table.row_count == 0:
            status_label.update("[yellow]No facts to copy[/yellow]")
            return

        # Gather rows with column headers
        header_row = "\t".join([col[0] for col in _COLUMNS])
        rows = []
        for i in range(table.row_count):
            row_data = table.get_row_at(i)
            rows.append("\t".join(str(cell) for cell in row_data))

        content = header_row + "\n" + "\n".join(rows)

        # Try multiple methods to copy to clipboard
        result = self._copy_to_clipboard(content)
        if result[0] == "clipboard":
            status_label.update(f"[green]Copied {table.row_count} fact(s) to clipboard[/green]")
        elif result[0] == "file":
            status_label.update(f"[yellow]Saved {table.row_count} fact(s) to: {result[1]}[/yellow]")
        else:
            status_label.update("[red]Failed to copy facts[/red]")

    def _copy_to_clipboard(self, content: str) -> tuple:
        """Try multiple methods to copy content to clipboard.

        Returns: (method, path_or_none) where method is "clipboard", "file", or "failed"
        """
        # Method 1: Try pyperclip
        if pyperclip:
            try:
                pyperclip.copy(content)
                return ("clipboard", None)
            except Exception as e:
                pass

        # Method 2: Try xclip (Linux) - only if DISPLAY is set
        if os.environ.get("DISPLAY"):
            try:
                proc = subprocess.Popen(
                    ["xclip", "-selection", "clipboard"],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                proc.communicate(content.encode("utf-8"), timeout=2)
                if proc.returncode == 0:
                    return ("clipboard", None)
            except Exception:
                pass

        # Method 3: Try xsel (Linux) - only if DISPLAY is set
        if os.environ.get("DISPLAY"):
            try:
                proc = subprocess.Popen(
                    ["xsel", "--clipboard", "--input"],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                proc.communicate(content.encode("utf-8"), timeout=2)
                if proc.returncode == 0:
                    return ("clipboard", None)
            except Exception:
                pass

        # Fallback: Write to a temp file
        try:
            import tempfile
            with tempfile.NamedTemporaryFile(mode="w", suffix=".tsv", delete=False) as f:
                f.write(content)
                temp_path = f.name
            return ("file", temp_path)
        except Exception:
            pass

        return ("failed", None)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    argv = sys.argv[1:sys.argv.index("--ros-args")] \
        if "--ros-args" in sys.argv else sys.argv[1:]

    parser = argparse.ArgumentParser(description="KnowledgeCore TUI viewer")
    parser.add_argument(
        "-r", "--rate",
        default=1.0,
        type=float,
        help="Refresh rate in Hz (default: 1.0)",
    )
    args = parser.parse_args(argv)

    fetcher = KBFetcher(period=1.0 / args.rate)
    fetcher.start()

    app = KBApp(fetcher=fetcher)
    try:
        app.run()
    finally:
        fetcher.stop()


if __name__ == "__main__":
    main()
