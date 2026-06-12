from __future__ import annotations

import importlib.resources


HTML_PAGE = (
    importlib.resources.files("rebotarm_dashboard.status_panel_assets")
    .joinpath("index.html")
    .read_text(encoding="utf-8")
)
