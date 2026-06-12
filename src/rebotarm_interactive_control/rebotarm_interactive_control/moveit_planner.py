import sys

from rebotarm_motion import moveit_planner as _module

sys.modules[__name__] = _module
