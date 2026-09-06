# Upstream MuJoCo snapshot localization

This directory is a byte-preserved comparison snapshot of
`huangbinai/robotarm_ros2` at
`fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`.

It is intentionally outside the default `src/` tree and is not added as a
second colcon package. The default simulation launch uses a small process
boundary that prepends this snapshot's Python source to the existing ROS
overlay and execs the pinned MuJoCo virtual environment. Do not source a second
snapshot `install/` overlay; use `mujoco_moveit_sim.launch.py` or the P1
comparison runner.

The package metadata declares Apache-2.0 but the pinned checkout has no root
`LICENSE`, `COPYING` or `NOTICE` file. Public redistribution of this exact
snapshot is authorized by the upstream owner as recorded in the repository root
`THIRD_PARTY_NOTICES.md`; see `UPSTREAM_PROVENANCE.json` for the pinned source
and scope. This authorization does not extend to a different upstream revision.

The package keeps non-package support inputs required by the upstream converter
and motor controller: `src/rebotarm_bringup/config/arm.yaml`,
`src/rebotarm_bringup/config/gripper.yaml`, and the authoritative
`src/rebotarm_moveit_config/config/rebotarm.urdf`. The copied launch/documentation
files exist only so the upstream tests retain their original repository-relative
contracts; they are not discovered by the current colcon workspace.

The upstream replay test currently has one known test-only defect: pytest 7.4.4
cannot recursively compare a mapping whose values are tuples through a single
`pytest.approx` call. This does not change the source snapshot or runtime model.
