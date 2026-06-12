# rebotarm_interactive_control

Compatibility package for legacy reBotArm interactive-control imports.

## Recommended entrypoints

Use the current bringup launch files and web dashboard workflows. RViz end-effector dragging is provided by the MoveIt MotionPlanning wrapper launches in `rebotarm_bringup`.

## Transitional / legacy entrypoints

Some legacy internal launches and nodes remain only for compatibility during refactoring. Do not use them as operator entrypoints.

New code should depend on the split packages directly, not this compatibility package.
