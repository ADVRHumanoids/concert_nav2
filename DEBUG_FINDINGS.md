# `concert_navigation` — Debug Findings (pre-fix audit)

> Static review of the package against the **installed** stack. Originally written
> *before* launching on the real robot; **updated 2026-06-04 after a live bring-up**
> — A1, A5 and the newly-found **A10** (`width/height` crash) are now **fixed and
> verified live on the robot** (see [§E](#e-runtime-verification-live-on-the-robot)).
> The two LiDAR-merger launch files were corrected earlier for single-lidar support
> — see [§B](#b-already-fixed).
>
> **Environment verified:** ROS 2 **Jazzy**, Nav2 **1.3.4**, BehaviorTree.CPP
> **4.6.2** (`/opt/ros/jazzy`).
>
> **Method:** read every launch/config/BT file in the package; cross-checked
> param names, plugin class names, polygon/format syntax and BT format against the
> installed Nav2 share files and headers; compiled a small BT.CPP test to settle
> the XML-format question; read the robot URDF for sensor mount heights.

## Severity legend
- 🔴 **HIGH** — will break or very likely break navigation at runtime.
- 🟠 **MEDIUM** — wrong/fragile behavior, silent foot-gun, or breaks a documented workflow.
- 🟡 **LOW** — cleanliness, dead config, deprecation warning, or non-fatal inconsistency.

---

## Summary table

| ID | Sev | File | Problem |
|----|-----|------|---------|
| A1 | 🔴 **FIXED** | `config/local_costmap.yaml` | Empty local costmap (robot blind). Scan plane measured **z=−0.063 m** in `odom` → discarded by *both* the `VoxelLayer` z-window **and** the two `min_obstacle_height` gates (per-source **+ layer-level**, default 0.0). Now `ObstacleLayer` + `min_obstacle_height: -0.3` at **both** levels. Verified live (§E). |
| A2 | 🟡 | `behavior_tree/behavior.xml` | `<root>` missing `BTCPP_format="4"` (BT.CPP 4 deprecation warning) |
| A3 | 🟡 | `config/controller.yaml` | `AckermannConstraints` block is dead config under `motion_model: "Omni"` |
| A4 | 🟡 | `launch/path_planner.launch.py` | `bt_navigator` activated before the servers it calls (lifecycle order) |
| A5 | 🟡 **FIXED** | `config/local_costmap.yaml` | Local was `VoxelLayer`, global `ObstacleLayer` for the same 2D `/scan`. Local switched to `ObstacleLayer` (parity) as part of the A1 fix. |
| A6 | 🟡 | `config/recovery.yaml` | `assisted_teleop` behavior loaded but never used by the BT |
| A7 | 🟡 | `config/planner_server.yaml` + BT | Path smoothed twice (Smac internal smoother + `smoother_server`) |
| A8 | 🟡 | `config/collision_monitor.yaml` | `PolygonStop` margin (~6 cm) ≈ full stopping distance at `vx_max` |
| A9 | 🟡 | `package.xml` | `description`/`license`/`maintainer` left as `TODO`/`user` |
| A10 | 🔴 **FIXED** | `config/local_costmap.yaml` | **Runtime crash.** `width: 5.0`/`height: 5.0` are doubles but Nav2 declares them `int` → `controller_server` aborts on configure (`InvalidParameterTypeException`, exit −6). Changed to `5`/`5`. Verified live (§E). |
| A11 | 🟡 | `config/collision_monitor.yaml` | `scan` source sets `min_height`/`max_height`, but Nav2 **`Scan` sources ignore height** (only `PointCloud` sources read it — `scan.hpp` has no height field). Dead config; the safety gate is **not** blind. `ARCHITECTURE.md §13` over-states it. |
| A12 | 🟡 | `config/planner_server.yaml` | SmacPlanner2D startup warning: `inflation_radius 0.7` < half the footprint's largest cross-section (~0.79 m) → slower collision checking. Bump `inflation_radius ≥ 0.8` on both costmaps. |
| C1 | 🟠 | `concert_localization/localization.launch.py` | README uses `map_file:=` but launch only accepts `map_name:=` → user's map silently ignored |
| C2 | 🟠 | `concert_localization` (launch + `amcl_config.yaml`) | `use_sim_time` defaults **true** → on the real robot AMCL/TF break unless overridden |
| C3 | 🟠 | `concert_localization/config/amcl_config.yaml` | `set_initial_pose: true` at `(0,0,0)` → starts mislocalized if robot isn't at map origin |
| C4 | 🟡 | `concert_mapping/map_saver.launch.py` + localization | Two nodes both named `lifecycle_manager_localization` (collision if co-run) |

Verified **non-issues** (looked suspicious, are actually correct) are listed in [§D](#d-verified-non-issues) so they don't get "fixed" by mistake.

---

## A. Findings in `concert_navigation` (this package)

### A1 🔴 **FIXED** — Empty local costmap: scan plane below the costmap's height gates
The merged `/scan` plane was being discarded by the local costmap, so it never marked
obstacles (robot blind; this is the README *"local costmap is empty"* entry).

**Measured live** (robot at rest): `tf2_echo odom VLP16_lidar_back_base_link` → translation
z = **−0.063 m**. The scan plane sits **6 cm below the `odom` origin** because the lidar is
`−0.0627` under `mobile_base` and wheel odometry puts `odom` at `base_link` (identity), not
at the ground. A standalone rclpy node re-projecting `/scan` into `odom` confirmed ~121
returns land inside the 5×5 m grid within `obstacle_max_range`, all at z=−0.063 → they
*should* mark.

Two independent filters were rejecting that sub-zero plane:
1. **`VoxelLayer` z-window** `[origin_z 0.0 … +0.80] m` (`z_voxels 16 × 0.05`) — voxelises in
   the costmap frame, so z=−0.063 → negative voxel index → dropped.
2. **`min_obstacle_height` — and there are TWO of them** (the non-obvious part):
   - `obstacle_layer.scan.min_obstacle_height` (per-source) → feeds the observation buffer.
   - `obstacle_layer.min_obstacle_height` (**layer-level**, default **0.0**) → gates the
     **marking loop**. Points pass the buffer but are silently dropped here, **with no error
     logged** — so the costmap looks perfectly healthy yet paints nothing.

- **Fix (applied):** local costmap → `ObstacleLayer` (drops the z-window; parity with
  global, see A5) **and** `min_obstacle_height: -0.3` at **both** the layer level and the
  `scan` source level. The same `min_obstacle_height: -0.3` was added to
  `global_costmap.yaml` (its obstacle layer was blind for the identical reason — the height
  filter runs in the global frame, and the buffer's `global_frame`/`min_obstacle_height`
  defaults to `0.0`).
- **Verified live:** raising the layer-level min from `0.0` → negative took the local
  costmap from **0** marks to **86 lethal + 2889 inflated** cells instantly (see §E).

### A2 🟡 BT XML missing `BTCPP_format="4"`
`behavior_tree/behavior.xml:1`:
```xml
<root main_tree_to_execute="MainTree">
```
All 12 default Nav2 Jazzy trees use `<root BTCPP_format="4" main_tree_to_execute="MainTree">`.
**Confirmed non-fatal:** I compiled a BT.CPP 4.6.2 test — a tree without the
attribute loads but prints:
```
Warnings: The first tag of the XML (<root>) should contain the attribute [BTCPP_format="4"]
Please check if your XML is compatible with version 4.x of BT.CPP
```
- **Fix (not applied):** add `BTCPP_format="4"` to the root tag.

### A3 🟡 Dead `AckermannConstraints` under an Omni motion model
`config/controller.yaml:56,64-65`: `motion_model: "Omni"` but the file still sets
```yaml
AckermannConstraints:
  min_turning_r: 0.2
```
which MPPI only reads for the Ackermann model. Harmless but misleading.
- **Fix (not applied):** delete the `AckermannConstraints` block.

### A4 🟡 Lifecycle activation order: `bt_navigator` before its servers
`launch/path_planner.launch.py:119-125` — `node_names` order is:
`planner_server, controller_server, recoveries_server, bt_navigator,
velocity_smoother, collision_monitor, smoother_server`.
`bt_navigator` (4th) is activated before `smoother_server` (7th), which it calls
via `SmoothPath`. Not fatal (the BT only calls servers on a goal, by which time
all are active), but Nav2's own bringup activates `bt_navigator` *after* the
action servers. Best-practice deviation.
- **Fix (not applied):** move `bt_navigator` to the end of the `node_names` list.

### A5 🟡 **FIXED** — Inconsistent obstacle layer between costmaps
Global used `ObstacleLayer`, local used `VoxelLayer`, for the **same** 2D
`LaserScan` `/scan` — the 3D voxel layer added the z-fragility of A1 with no benefit.
**Fixed:** local switched to `ObstacleLayer`, so both costmaps are now consistent
(done together with the A1 fix).

### A6 🟡 `assisted_teleop` loaded but unused
`config/recovery.yaml:10` registers `assisted_teleop`, but `behavior.xml` never
invokes it (recoveries used: clear costmaps, Spin, Wait, BackUp). Dead plugin
load. Harmless.

### A7 🟡 Path is smoothed twice
`planner_server.yaml` enables the SmacPlanner2D **internal** smoother, and the BT
then runs `SmoothPath` via `smoother_server` (`simple_smoother`). Redundant CPU;
usually you keep one. Not wrong, just wasteful.

### A8 🟡 `PolygonStop` margin ≈ stopping distance
`config/collision_monitor.yaml:17` — `PolygonStop` is `1.5 × 1.0 m`
(x ±0.75, y ±0.5) vs. footprint x ±0.685 / y ±0.4, i.e. only **~6 cm** ahead of
the footprint. At `vx_max 0.5 m/s` with `max_decel 2.0 m/s²`, stopping distance is
`v²/2a ≈ 6.25 cm`. The hard-stop zone is about one stopping-distance deep, leaving
no slack. The `FootprintApproach` (TTC 1.2 s) and `PolygonSlow` mitigate this, but
consider enlarging `PolygonStop`. Tuning, not a crash.

### A9 🟡 Package metadata placeholders
`package.xml:6-8` — `description` = `TODO`, `license` = `TODO`, `maintainer` =
`user`. Cosmetic.

### A10 🔴 **FIXED** — `controller_server` crash on `width`/`height` type
`config/local_costmap.yaml` had `width: 5.0` / `height: 5.0`. Nav2 declares costmap
`width`/`height` as **integers** (canonical `nav2_params.yaml` uses `3`/`3`), so the
doubles throw on configure and take down the whole bring-up:
```
parameter 'height' has invalid type: ... {integer}, setting it to {double} is not allowed
controller_server: process has died ... exit code -6
```
The local costmap lives *inside* `controller_server`, so this aborted lifecycle bring-up
entirely (lifecycle manager: *"Failed to bring up all requested nodes"*). **Not in the
original static review** — only surfaced at runtime. Global costmap is unaffected (it does
not set `width`/`height`).
- **Fix (applied):** `width: 5`, `height: 5` (integers). Verified live (§E).

### A11 🟡 Dead height filter on the collision-monitor scan source
`config/collision_monitor.yaml` `scan` source sets `min_height: 0.15` / `max_height: 2.0`.
Nav2's `Scan` source is purely 2D and **ignores** these — `min_height_`/`max_height_` exist
only in `nav2_collision_monitor/pointcloud.hpp`, never `scan.hpp`. So they are dead config
(harmless today), **but** had they been honoured they would have blinded the safety gate
(scan plane is z=−0.063, below `0.15`). `ARCHITECTURE.md §13` incorrectly lists this as an
active `0.15–2.0 m` filter (corrected there). **Trap:** switching this source to a
`PointCloud` type makes those heights apply and the monitor goes blind.
- **Fix (optional):** drop the two height lines (or keep, but know they're scan-irrelevant).

### A12 🟡 Smac `inflation_radius` below the planner's optimum
At startup SmacPlanner2D logs: *"Inflation layer … not set sufficiently for optimized
non-circular collision checking … set the inflation radius to be at MINIMUM half of the
robot's largest cross-section."* Footprint 1.37 × 0.8 m → largest cross-section (diagonal)
≈ 1.59 m → half ≈ **0.79 m**, but `inflation_radius` is **0.70** in both costmaps. The
planner still runs, just with slower collision checking.
- **Fix (optional):** raise `inflation_radius` to ≥ 0.8 (trades a little path clearance for
  speed).

---

## B. Already fixed (previous step)
`launch/master_lidar_conversion_fuse.launch.py` and
`launch/master_cloud_multi_merger.launch.py` were reworked to be single-lidar
robust and parameterized (`use_front_lidar` / `use_back_lidar`). That step also
fixed: a non-existent `IncludeLaunchDescription` target, a wrong cloud topic
(`…/points` → `…/velodyne_points`), and an invalid `destination_frame`
(`VLP16_lidar_back` → `<scanner>_base_link`). See `ARCHITECTURE.md` §7 and §14.

The **root cause** they addressed (documented here for completeness): the
`ira_laser_tools::laserscan_multi_merger` (a) blocks in its constructor until
*every* `laserscan_topics` entry is advertised, and (b) only publishes once every
subscribed topic delivered a fresh scan — so any dead lidar in the list freezes
`/scan` and the whole stack.

---

## C. Cross-package issues that affect this stack's bring-up

These live in sibling packages but directly break the documented `concert_navigation`
workflow, so they belong in a "before launching" audit.

### C1 🟠 README map argument doesn't exist on the launch
`README.md:145` shows
`localization.launch.py map_file:=/absolute/path/to/map.yaml`, but
`concert_localization/launch/localization.launch.py` declares **`map_name`** (and
`amcl_config`), not `map_file`, and hard-wires
`concert_mapping/maps/<map_name>/myworld.yaml`. Passing `map_file:=` is silently
ignored → you always load the default `map_test` map, not the one you intended.

### C2 🟠 `use_sim_time` defaults to **true** for localization
`concert_localization/launch/localization.launch.py:12-13` and
`config/amcl_config.yaml:4` default `use_sim_time` to `true`. On the real robot
(no `/clock`) AMCL will stamp/expect sim time and fail to localize/transform
unless you remember `use_sim_time:=false` (the README does pass it, but the unsafe
default is a foot-gun). `concert_navigation`'s own launches default to `false`,
so the two halves disagree.

### C3 🟠 AMCL forces an initial pose at the origin
`concert_localization/config/amcl_config.yaml:14-19` — `set_initial_pose: true`
with `initial_pose {0,0,0}`. If the robot doesn't physically start at the map
origin, AMCL begins mislocalized and won't wait for a `2D Pose Estimate`. For
field use, prefer `set_initial_pose: false` (or set the true pose).

### C4 🟡 Duplicate lifecycle-manager node name
`concert_mapping/map_saver.launch.py` names its manager
`lifecycle_manager_localization`, the same name used by
`concert_localization/localization.launch.py`. If both run together you get two
nodes with the same name. (Also semantically wrong — it manages `map_saver_server`.)

---

## D. Verified NON-issues (do **not** "fix" these)

Checked against installed Jazzy / Nav2 1.3.4 — these are correct as written:

- **`collision_monitor` polygon `points` as a string** `"[[x,y],...]"` — correct
  for Jazzy. The shipped `nav2_collision_monitor/params/collision_monitor_params.yaml`
  uses the exact same string form with `min_points`. (Do **not** convert to a flat
  array.)
- **MPPI critic names** in `controller.yaml` — all 10 exist in
  `nav2_mppi_controller/critics.xml` (`ConstraintCritic, CostCritic, GoalCritic,
  GoalAngleCritic, PathAlignCritic, PathFollowCritic, PathAngleCritic,
  PreferForwardCritic, TwirlingCritic, VelocityDeadbandCritic`).
- **`nav2_behaviors` plugin types** in `recovery.yaml` — `Spin, BackUp,
  DriveOnHeading, Wait, AssistedTeleop` all present in `behavior_plugin.xml`;
  `max_rotational_vel/min_rotational_vel/rotational_acc_lim/simulate_ahead_time`
  are valid server-level params (in `spin.hpp`/`drive_on_heading.hpp`).
- **Double-nested costmap YAML** (`local_costmap: local_costmap: ros__parameters:`)
  — this is the canonical Nav2 format, not a typo.
- **`enable_stamped_cmd_vel: false`** everywhere — valid Jazzy param
  (`nav2_util/twist_(sub|pub)`); consistent with the base reading plain `Twist`.
- **`cmd_vel` remaps** (controller/recoveries/velocity_smoother → `cmd_vel_nav`)
  and the `cmd_vel_nav → cmd_vel_smoothed → omnisteering/cmd_vel` chain — correct.
- **AMCL/slam `base_link_projected` base frame** — intentional 2D ground-projection
  frame from the URDF; requires `robot_state_publisher` up (external dependency,
  not a bug).
- **Map files** — both `concert_mapping/maps/myworld.yaml` and
  `maps/map_test/myworld.yaml` (+ `.pgm`) exist; the localization code comment
  *"not already exists"* is stale.

---

## E. Runtime verification (live on the robot)

Bring-up on **2026-06-04** (ROS 2 Jazzy), stack = XBot2 + both VLP16 drivers +
`concert_odometry` + back-only `master_lidar_conversion_fuse` (`/scan` @ 10 Hz) +
`path_planner` (plus a dummy identity `map→odom` so the lifecycle manager completes
without real localization):

- **A10 (crash) — confirmed & fixed.** Before: `controller_server` died with the
  `InvalidParameterTypeException` above. After `width/height → int`: it configures *and*
  activates; lifecycle manager reports *"Managed nodes are active."*
- **A1 (empty costmap) — root-caused & fixed live.** `/scan` healthy (480 finite returns,
  nearest 1.48 m, all at z=−0.063 in `odom`); ~121 returns inside the grid within
  `obstacle_max_range`. Costmap still painted **0** marks with `min_obstacle_height: -0.3`
  on the *source only*. Setting the **layer-level** `obstacle_layer.min_obstacle_height`
  negative → **86 lethal + 2889 inflated** cells immediately ⇒ both filters are required.
- **A11 — collision monitor is not blind** (the `Scan` source ignores height; verified
  `scan.hpp` has no height field).
- **Global costmap** stays void without real localization (it is `map`-framed); with only a
  dummy `map→odom` it logs *"sensor origin out of map bounds"*. Expected — run real
  `concert_localization` / `slam_toolbox` to fill it. The **local** costmap is `odom`-framed
  and works with no localization at all.

Tools used: `ros2 lifecycle get`, `ros2 param get/set`, `ros2 topic echo --field` /
`topic delay` / `topic info --verbose`, `tf2_echo`, and a standalone rclpy node that
re-projected `/scan` into `odom` to prove the data was markable independently of Nav2.

---

## Suggested fix order — status
1. ✅ **A1 / A5 / A10** (local costmap empty + crash) — **done & verified live**; the
   layer-level `min_obstacle_height` gate was the non-obvious part.
2. ⏭️ **C2 / C1 / C3** — still open; make the real-robot localization bring-up safe
   (sim-time default, map arg, initial pose) before the first mapped run.
3. ⏭️ **A12** (inflation_radius), **A4** (lifecycle order), **A2** (BT format) — quick wins.
4. ⏭️ **A3, A6, A7, A8, A9, A11, C4** — cleanup/tuning.
