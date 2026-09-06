# P1 Upstream MuJoCo Migration Design

**Status:** Approved direction; implementation starts with a pinned read-only snapshot.

## Goal

Use `huangbinai/robotarm_ros2`'s completed MuJoCo stack as the primary P1 migration candidate, while preserving the current package-owned implementation as a fallback and comparison baseline.

## Source boundary

- Repository: `https://github.com/huangbinai/robotarm_ros2.git`
- Branch: `main`
- Pinned snapshot: `fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`
- Candidate files: `src/rebotarm_simulation/`, its `models/rebotarm/`, config, launch, tests, and MuJoCo documentation.
- The upstream package declares Apache-2.0, but the root `LICENSE` file is not currently available; copied files remain provisional until file-level license evidence is recorded.
- HJX remains behavior-only observation and is not a migration source.

## Architecture

The snapshot is kept outside the default ROS package path and never loaded by the real hardware launch. A differential report compares model generation, health/headless checks, viewer, motor control, collision/contact behavior, ROS 2 endpoints, metrics, and tests. Only selected capabilities are adapted into the current `rebotarm_simulation` ownership boundary; P0 hardware safety, current metrics/tolerance semantics, and action/service compatibility remain authoritative.

## P1 work packages

1. Pin the upstream snapshot and complete source/file/license inventory.
2. Run upstream health, headless, viewer, model-contract, motor-control, collision, and ROS checks independently.
3. Localize selected upstream model/converter/control capabilities and preserve current API and real/sim mutual exclusion.
4. Integrate timeout and execute-loop failure handling.
5. Complete effort/velocity/acceleration/jerk consistency checks.
6. Calibrate joint4-6 tracking, collision, gripper contact, and grasp quality against current geometry and measured behavior.

## Acceptance rule

An upstream test passing proves upstream behavior only. A P1 checkbox is marked complete only after the localized current package passes its own tests and the differential evidence identifies any remaining calibration or interface gaps.
