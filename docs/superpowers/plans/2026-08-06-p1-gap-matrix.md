# P1 Current/Upstream Gap Matrix Implementation Plan

> **For agentic workers:** Inline execution in this session; no subagent dispatch.

**Goal:** Create an auditable current/upstream MuJoCo capability gap matrix and record only evidence-backed P1 progress.

**Architecture:** The matrix is documentation owned by `docs/`, while a small repository test validates its schema and required capability rows. Upstream HJX material is limited to behavior-level observation and is never used as a code or asset source.

**Tech Stack:** Markdown, Python/pytest, existing MuJoCo/ROS 2 evidence.

---

### Task 1: Define the matrix contract

**Files:**
- Create: `tests/test_mujoco_gap_matrix.py`
- Create: `docs/mujoco_gap_matrix.md`

- [ ] Write a test that requires the matrix header, all P1 capability rows, and explicit `defer`/`missing` decisions for unfinished capabilities.
- [ ] Run the test and confirm it fails because the matrix is absent.
- [ ] Write the evidence-backed matrix with current evidence, upstream observation, decision, owner, and next action.
- [ ] Run the focused test and confirm it passes.

### Task 2: Update project state and verify

**Files:**
- Modify: `Agent/PROJECT_STATUS.md`
- Modify: `Agent/MEMORY.md`

- [ ] Mark only the gap-matrix checkbox complete and attach the new document as evidence.
- [ ] Run layering, full tests, compileall, and diff checks.
- [ ] Refresh `Agent/STATE.json` with `update_state.py --event verified` and record the next P1 queue item.
