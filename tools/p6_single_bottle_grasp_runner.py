#!/usr/bin/env python3
"""Compatibility entry point for the installed single-bottle grasp feature."""

from pathlib import Path
import sys


_ROOT = Path(__file__).resolve().parents[1]
for _package in ("rebotarm_motion", "rebotarm_vision"):
    _source = _ROOT / "src" / _package
    if str(_source) not in sys.path:
        sys.path.insert(0, str(_source))

from rebotarm_vision.single_bottle_grasp import main


if __name__ == "__main__":
    main()
