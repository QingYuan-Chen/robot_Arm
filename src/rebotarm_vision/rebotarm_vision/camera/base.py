from __future__ import annotations

from typing import Optional, Protocol

import numpy as np


class CameraDriver(Protocol):
    def open(self) -> None:
        ...

    def warmup(self, frames: int) -> bool:
        ...

    def get_frame(self) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        ...

    def close(self) -> None:
        ...
