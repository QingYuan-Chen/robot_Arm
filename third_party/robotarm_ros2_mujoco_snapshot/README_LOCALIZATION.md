# Upstream MuJoCo snapshot localization

This directory is a byte-preserved comparison snapshot of
`huangbinai/robotarm_ros2` at
`fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`.

It is intentionally outside the default `src/` tree. Do not add it to the normal
colcon workspace and do not source it together with the current
`rebotarm_simulation` installation. Use the P1 comparison runner with an explicit
`PYTHONPATH` and the pinned MuJoCo virtual environment.

The snapshot is private and provisional because the package metadata declares
Apache-2.0 but the pinned checkout has no root `LICENSE`, `COPYING` or `NOTICE`
file. See `UPSTREAM_PROVENANCE.json` and the P1 evidence reports before any
distribution or source publication.

The package keeps non-package support inputs required by the upstream converter
and motor controller: `src/rebotarm_bringup/config/arm.yaml`,
`src/rebotarm_bringup/config/gripper.yaml`, and the authoritative
`src/rebotarm_moveit_config/config/rebotarm.urdf`. The copied launch/documentation
files exist only so the upstream tests retain their original repository-relative
contracts; they are not discovered by the current colcon workspace.

The upstream replay test currently has one known test-only defect: pytest 7.4.4
cannot recursively compare a mapping whose values are tuples through a single
`pytest.approx` call. This does not change the source snapshot or runtime model.
