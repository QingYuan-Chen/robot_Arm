import sys

from rebotarm_dashboard import arm_control_client as _module

sys.modules[__name__] = _module
