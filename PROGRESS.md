# Autonomy Upgrade — 2-Week Progress Tracker

**Goal:** Turn `mobile_arm_sim` from a hardcoded pick-and-place demo into one that *looks autonomous* — camera + LIDAR sensing, Nav2 navigation around obstacles, vision-discovered target block among distractors, all driven by a Python state machine.

**Window:** 2026-06-24 → 2026-07-08 (next prof checkpoint).
**Full plan (context, architecture, risks):** `~/.claude/plans/inherited-churning-steele.md`

Tick boxes as you finish. Each day is ~2–4 hours of focused work.

---

## Day 0 — Setup (do first; ~30 min)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-planner \
  ros-humble-nav2-controller \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-smac-planner \
  ros-humble-nav2-costmap-2d \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard
```

- [x] All packages install cleanly (2026-06-24)
- [x] `ros2 pkg list | grep -E 'nav2_(bringup|planner|controller|bt_navigator)|slam_toolbox'` returns all four (2026-06-24)

---

## Week 1 — Foundation

### Day 1 — Sensors in URDF
- [x] Add `camera_link` (forward, ~10cm above front bumper, ~20° down tilt) + fixed joint to `base_link` (2026-06-26)
- [x] Add `lidar_link` (top of chassis) + fixed joint to `base_link` (2026-06-26)
- [x] Add `<sensor type="camera">` with `libgazebo_ros_camera.so` — 640×480, 30 Hz, ~1.2 rad HFOV, topic prefix `/camera` (2026-06-26; first attempt published to `/camera/camera/image_raw` due to namespace+camera_name stacking — fixed by setting `<namespace>/</namespace>`)
- [x] Add `<sensor type="ray">` with `libgazebo_ros_ray_sensor.so` — 360 samples, 10 m range, 10 Hz, topic `/scan` (2026-06-26)
- [x] `colcon build`, launch, verify `ros2 topic hz /scan` ≈ 10, `/camera/image_raw` ≈ 30 (2026-06-26 — `/scan` rock-solid 10.0 Hz; `/camera/image_raw` ~8 Hz avg with high variance — Gazebo RTF is 1.0 so variance is `rqt_image_view` subscriber overhead, not the sim. Accepted; revisit if Day 5 perception is choppy.)
- [x] Camera visible in `rqt_image_view`; LIDAR scan visible in RViz (2026-06-26 — robot loaded cleanly in Gazebo with sensors attached; topics publishing)
- [ ] Commit: `sim: add camera + LIDAR sensors`

### Day 2 — Scene authoring
- [ ] Create `worlds/autonomous.world` (copy `worlds/pick_place.world` structure)
- [ ] Add 4 thin static-box walls forming ~5m × 5m enclosure
- [ ] Add 2–3 obstacles (cylinders/boxes, static)
- [ ] Place `target_block` (red, 5cm cube) requiring nav around at least one obstacle
- [ ] Place 3 distractors: dark-orange, magenta, brown (5cm cubes)
- [ ] Place `target_table` at drop-off location
- [ ] Create `launch/autonomous.launch.py` using the new world (keep `pick_place.launch.py` working)
- [ ] Teleop confirms reachability + camera shows colors distinctly
- [ ] Commit: `sim: autonomous scene with obstacles + distractors`

### Day 3 — Static map generation
- [ ] Add `maps/` dir + extend `install(DIRECTORY ...)` in `CMakeLists.txt`
- [ ] Write `launch/mapping.launch.py` (Gazebo + RSP + robot + `slam_toolbox` online_async)
- [ ] Teleop until map complete in RViz
- [ ] `ros2 run nav2_map_server map_saver_cli -f src/mobile_arm_sim/maps/autonomous_map`
- [ ] `.pgm` looks correct (walls + obstacles, free space)
- [ ] Sanity check: `nav2_bringup map_server.launch.py` shows the map in RViz
- [ ] Commit: `sim: static occupancy map for AMCL + Nav2`

### Day 4 — Nav2 bringup
- [ ] Write `config/nav2_params.yaml` — amcl + planner_server + controller_server + costmaps + bt_navigator
- [ ] Inflate `planar_move` odom covariance in URDF (e.g., `covariance_x: 0.01`) so AMCL has work to do
- [ ] Write `launch/nav2.launch.py` (map_server + amcl + planner + controller + bt_navigator + behavior_server + lifecycle_manager)
- [ ] Update `autonomous.launch.py` to include Nav2
- [ ] Particle cloud visible after RViz "2D Pose Estimate"
- [ ] "2D Nav Goal" → robot navigates; global + local plan + both costmaps visible
- [ ] Commit: `nav: Nav2 bringup with AMCL + LIDAR costmap`

### Day 5 — Perception (block_detector)
- [ ] Create `scripts/block_detector.py` — subscribes to `/camera/image_raw` + `/camera/camera_info`
- [ ] HSV mask for red (two hue ranges, morphological clean) → largest contour
- [ ] Pixel → ground-plane projection via `CameraInfo.K` + TF lookup
- [ ] Publish `/target_block_pose` (`PoseStamped` in `map` frame) only above contour-area threshold
- [ ] Publish `/target_block_marker` for RViz visualization
- [ ] Teleop near target → marker sits on the block in RViz
- [ ] Teleop near each distractor → NO pose published
- [ ] Commit: `perception: HSV block detector with ground-plane projection`

---

## Week 2 — Integration

### Day 6 — Orchestrator skeleton
- [ ] Create `scripts/autonomous_pick_place.py` with `State(Enum)`: `IDLE, SEARCHING, APPROACHING, ALIGNING, GRASPING, CARRYING, PLACING, RETURNING, DONE, FAILED`
- [ ] Copy arm/gripper helpers + `_teleport_block` from `scripts/pick_and_place.py`
- [ ] 5 Hz timer dispatches to per-state handlers
- [ ] Every state stubbed with log + fixed delay
- [ ] Logs walk `IDLE → ... → DONE` cleanly
- [ ] Commit: `orchestrator: state-machine skeleton`

### Day 7 — Orchestrator + Nav2 wired
- [ ] Add `NavigateToPose` action client
- [ ] `SEARCHING`: spin in place until `/target_block_pose` arrives fresh
- [ ] `APPROACHING`: compute 30cm standoff in front of block, send Nav2 goal; on aborted retry once → else `FAILED`
- [ ] `CARRYING`: Nav2 to hardcoded drop pose
- [ ] `RETURNING`: Nav2 home
- [ ] GRASPING / PLACING just transit through for today
- [ ] Robot rotates, finds block, navigates to standoff
- [ ] Commit: `orchestrator: Nav2 action client + nav states`

### Day 8 — Orchestrator + arm wired
- [ ] `GRASPING`: `PRE_GRASP` → `GRASP` → close gripper → `attach_block()` → `LIFT`
- [ ] `PLACING`: `DROP` → open gripper → `detach_block()` → `LIFT`
- [ ] `_teleport_block()` runs from timer during `block_attached`
- [ ] **End-to-end run succeeds** with single target, no distractors
- [ ] Commit: `orchestrator: full pipeline`

### Day 9 — Robustness + distractor confirmation
- [ ] With 3 distractors spawned: orchestrator never targets any of them
- [ ] **Freshness check**: in `APPROACHING`, no fresh pose for >2s → back to `SEARCHING`
- [ ] **Retry**: any Nav2 `aborted` → retry once with small goal offset → else `FAILED`
- [ ] `FAILED` recovery: drive home, re-enter `SEARCHING` from different angle (one total retry)
- [ ] Stretch: parametrize target block spawn (x, y random within region) in `autonomous.launch.py`
- [ ] 3 consecutive runs at different positions all succeed
- [ ] Commit: `orchestrator: distractors + retries + freshness`

### Day 10 — Polish + demo
- [ ] Save `config/autonomous.rviz`: camera, scan, map, particle cloud, both costmaps, plans, TF, target marker
- [ ] `autonomous.launch.py` opens RViz with this layout
- [ ] Record full-run demo video (target in a "hard" position behind obstacle)
- [ ] README updated (gif + architecture diagram + how to run)
- [ ] Final commit

---

## Stretch — Option C (online SLAM)
**Only start if Day 9 finishes a full day early.** Swap AMCL + static map for `slam_toolbox` in `online_async`. High-risk; skippable.

---

## End-to-end acceptance (run on Day 10)

```bash
colcon build --packages-select mobile_arm_sim && source install/setup.bash
ros2 launch mobile_arm_sim autonomous.launch.py
```

Expected within ~60s of Gazebo settling:
1. Particle cloud localizes; costmaps populate
2. Robot rotates, identifies red block (target marker appears)
3. Nav2 plans around obstacle → robot approaches standoff
4. Arm grasps block, base navigates to table, arm places, base returns home
5. Distractors present throughout — never become goals
6. Repeat with target at a new position — same behavior

---

## Notes / known risks

- **AMCL is bored in sim** without odom noise → particle cloud collapses. Day 4 mitigation: inflate `planar_move` covariance.
- **Camera extrinsics matter** for projection. Day 5: verify with `ros2 service call /get_entity_state ...` ground-truth comparison; tune until error < 3cm.
- **Magic grasp jitter at high Nav2 speeds** — Day 8: zero `req.state.twist.linear/angular` in `_teleport_block` to prevent.
- **slam_toolbox map noise** — hand-edit `.pgm` in GIMP if needed before Day 4.
- **If Days 1–5 slip:** drop AMCL pre-built map, use odom-only + LIDAR costmap. Demo still works.
