from __future__ import annotations

import cv2
import numpy as np

from .camera.gemini2_driver import Gemini2Config, Gemini2Driver


def _depth_to_u8(depth_mm: np.ndarray | None) -> np.ndarray | None:
    if depth_mm is None:
        return None
    valid = depth_mm[depth_mm > 0]
    if valid.size == 0:
        return np.zeros((depth_mm.shape[0], depth_mm.shape[1], 3), dtype=np.uint8)
    near = float(np.percentile(valid, 5))
    far = float(np.percentile(valid, 95))
    if far <= near:
        far = near + 1.0
    clipped = np.clip(depth_mm.astype(np.float32), near, far)
    normalized = ((clipped - near) / (far - near) * 255.0).astype(np.uint8)
    normalized[depth_mm == 0] = 0
    return cv2.applyColorMap(normalized, cv2.COLORMAP_JET)


def main() -> None:
    driver = Gemini2Driver(
        Gemini2Config(
            color_width=640,
            color_height=480,
            color_fps=30,
            enable_depth=True,
            depth_width=0,
            depth_height=0,
            depth_fps=30,
            frame_timeout_ms=1000,
            enable_align=False,
        )
    )

    driver.open()
    driver.warmup(15)

    cv2.namedWindow("Gemini2 Color", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("Gemini2 Depth", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            color_bgr, depth_mm = driver.get_frame(allow_partial=True)

            if color_bgr is not None:
                overlay = color_bgr.copy()
                stats = (
                    f"color min={int(color_bgr.min())} "
                    f"max={int(color_bgr.max())} mean={float(color_bgr.mean()):.1f}"
                )
                cv2.putText(
                    overlay,
                    stats,
                    (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("Gemini2 Color", overlay)

            depth_vis = _depth_to_u8(depth_mm)
            if depth_vis is not None:
                stats = (
                    f"depth min={int(depth_mm.min())} "
                    f"max={int(depth_mm.max())} mean={float(depth_mm.mean()):.1f}"
                )
                cv2.putText(
                    depth_vis,
                    stats,
                    (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("Gemini2 Depth", depth_vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
    finally:
        driver.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
