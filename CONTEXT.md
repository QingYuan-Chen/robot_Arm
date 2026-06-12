# reBotArm Context

## Terms

### Hardware Controller

The ROS2 layer that owns real motor communication and last-line execution
safety. In this repository, this is `rebotarmcontroller`.

### Point-to-Point Execution

Moving the robot from its current state to one target state. It is a motion
problem, not a teach replay problem. The expected output is a valid trajectory
with a safe final state.

### Teach Replay

Reproducing a recorded hand-guided trajectory. Raw teach data is input data; the
robot should execute a prepared and validated trajectory.

### Prepared Trajectory

The filtered, resampled, retimed, and checked trajectory derived from a teach
record. Real replay should prefer this prepared trajectory over raw samples.

### Operator Interaction

Human command input through web, keyboard, RViz marker, or gripper controls.
Operator interaction translates intent into ROS commands but does not own
hardware internals or teach replay algorithms.

### Dashboard

The web-facing UI and status API. It displays state and calls services but does
not own motion planning, teach replay algorithms, or motor SDK calls.

### Compatibility Layer

An old package or module path kept so existing launch files and imports do not
break immediately. In this repository, `rebotarm_interactive_control` is a
compatibility layer after the package split.
