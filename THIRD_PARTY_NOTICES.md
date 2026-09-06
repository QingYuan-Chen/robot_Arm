# Third-party notices

## MuJoCo upstream source

- **Upstream project:** `huangbinai/robotarm_ros2`
- **Source:** <https://github.com/huangbinai/robotarm_ros2>
- **Pinned revision:** `fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`
- **Historical scope:** the 57-file comparison snapshot formerly under
  `third_party/robotarm_ros2_mujoco_snapshot`, including code, configuration,
  MJCF/XML models, STL meshes, documentation, and tests. The comparison copy
  is now local-only and recoverable from Git history before this cleanup.
- **Current source:** upstream-derived implementation and resources remain in
  `src/rebotarm_simulation`; local modifications are tracked in Git. Removing
  the comparison copy does not remove this attribution or expand authorization.
- **Redistribution authorization:** the maintainer of this repository confirms
  that the upstream owner authorized public redistribution of this pinned
  snapshot for this project on 2026-08-13.

The pinned upstream checkout declares `Apache-2.0` in package metadata, but it
does not contain a root `LICENSE`, `COPYING`, or `NOTICE` file. The authorization
above is the basis for redistributing this exact snapshot; it does not grant a
license for any different upstream revision or for third-party material outside
the stated scope.

## Excluded local material

The following local-only directories and files are not part of this repository
release: untracked model checkpoints and weights, Python virtual environments, build and
install outputs, `third_party/reBotArm_control_py`,
`third_party/reBotArm_develop_hjx`, `third_party/rebotarm_mujoco_venv`, and
`third_party/robotarm_ros2_upstream`, the two historical simulation reference
directories, experiment evidence, and historical implementation plans.
The currently packaged `tools/yolo26s-seg.pt` remains tracked; its distribution
and installation workflow is separate from this historical-material cleanup.
