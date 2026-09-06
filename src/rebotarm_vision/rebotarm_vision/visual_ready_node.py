"""Compatibility entry point; ready-pose motion is owned by rebotarm_motion."""
import sys
from rebotarm_motion import visual_ready_node as _module

sys.modules[__name__] = _module
