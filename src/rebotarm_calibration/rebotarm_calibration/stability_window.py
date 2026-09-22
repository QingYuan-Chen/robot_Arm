"""Continuous pose stability; missing observations cannot bridge a long gap."""
import numpy as np
from .handeye_solver import rotation_angle_deg


class StabilityWindow:
    def __init__(self, duration, max_gap, translation, rotation):
        self.duration, self.max_gap = duration, max_gap
        self.translation, self.rotation = translation, rotation
        self.values = []

    def reset(self):
        self.values.clear()

    def add(self, stamp, pose):
        if self.values:
            delta = (stamp - self.values[-1][0]) / 1e9
            if delta <= 0 or delta > self.max_gap:
                self.reset()
        if any(np.linalg.norm(pose[:3, 3] - p[:3, 3]) > self.translation or
               rotation_angle_deg(p[:3, :3].T @ pose[:3, :3]) > self.rotation
               for _, p in self.values):
            self.reset()
        self.values.append((stamp, pose.copy()))
        return len(self.values) >= 3 and (stamp - self.values[0][0]) / 1e9 >= self.duration
