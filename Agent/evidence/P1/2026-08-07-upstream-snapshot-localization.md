# Upstream MuJoCo 完整本地快照证据

日期：2026-08-07  
来源：`/home/a/project/rebot_refer` clean checkout  
目标：将固定上游 MuJoCo package 纳入当前 Git，但保持默认 ROS package 和真实硬件路径不变。

## Snapshot identity

- remote：`https://github.com/huangbinai/robotarm_ros2.git`
- branch：`main`
- commit：`fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`
- target：`third_party/robotarm_ros2_mujoco_snapshot/`
- source diff：
  `diff -qr /home/a/project/rebot_refer/src/rebotarm_simulation third_party/robotarm_ros2_mujoco_snapshot/src/rebotarm_simulation`
  无输出，说明复制包字节内容一致。
- current default source：`src/rebotarm_simulation/` 未被 upstream snapshot 覆盖。

## Included scope

- 完整 `src/rebotarm_simulation/` runtime package、models/assets、config、launch 和 README；
- converter/motor control 所需的 `src/rebotarm_bringup/config/{arm.yaml,gripper.yaml}` 与
  `src/rebotarm_moveit_config/config/rebotarm.urdf`；
- upstream tests 所需的 repository-relative support files：`.gitattributes`、
  `docs/rebotarm_feature_commands.md`、两个 launch source；
- upstream `test_mujoco_*.py` 与 `test_urdf_to_mjcf.py`；
- `UPSTREAM_PROVENANCE.json` 记录来源、SHA-256 和 license evidence；
- `README_LOCALIZATION.md` 记录独立运行和 provisional license 边界。

## License boundary

`package.xml`/`setup.py` 声明 Apache-2.0，但 pinned checkout 根目录未发现
`LICENSE`、`COPYING` 或 `NOTICE`；快照状态保持 `PROVISIONAL`，只用于当前仓库私有
comparison，不对外分发。

## Verification

```text
tests/test_upstream_snapshot_provenance.py: 2 passed
package diff: clean (no output)
```

尚未把 snapshot 放入正常 `src/` 或 colcon package discovery；下一步是添加安全 runner，
再独立运行 snapshot health/headless 与 A/B comparison harness。
