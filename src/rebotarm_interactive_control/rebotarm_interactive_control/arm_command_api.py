import sys

from rebotarm_dashboard import arm_command_api as _module

sys.modules[__name__] = _module
