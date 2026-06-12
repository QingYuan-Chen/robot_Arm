from __future__ import annotations

from typing import Sequence

from ultralytics import YOLO


class YoloDetector:
    def __init__(
        self,
        model_path: str,
        device: str,
        conf_threshold: float,
        iou_threshold: float,
        use_world: bool,
        custom_classes: Sequence[str],
    ) -> None:
        self._model = YOLO(model_path)
        self._device = device
        self._conf_threshold = conf_threshold
        self._iou_threshold = iou_threshold
        self._open_vocab_enabled = False
        self._class_prompt_error = None
        if use_world and ("world" in model_path.lower() or "yoloe" in model_path.lower()):
            try:
                self._model.set_classes(list(custom_classes))
                self._open_vocab_enabled = True
            except Exception as exc:
                self._class_prompt_error = f"{type(exc).__name__}: {exc}"

    def infer(self, image_bgr):
        return self._model.predict(
            image_bgr,
            verbose=False,
            device=self._device,
            conf=self._conf_threshold,
            iou=self._iou_threshold,
        )

    @property
    def open_vocab_enabled(self) -> bool:
        return self._open_vocab_enabled

    @property
    def class_prompt_error(self) -> str | None:
        return self._class_prompt_error
