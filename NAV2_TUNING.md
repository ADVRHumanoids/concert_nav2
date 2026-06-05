# NAV2_TUNING.md — Behavior analysis & tuning recommendations

> **Status: ANALYSIS ONLY — no configuration has been changed.**
> This document explains *what* should change in the `concert_navigation` Nav2
> stack, *why*, and *with what evidence*, so we can decide and apply changes
> deliberately afterwards. Companion to [`ARCHITECTURE.md`](ARCHITECTURE.md)
> (how it is wired) and [`DEBUG_FINDINGS.md`](DEBUG_FINDINGS.md) (bugs already
> fixed). Targets **ROS 2 Jazzy / Nav2 1.3.x**.
>
> **Design decisions are now locked (2026-06-05) — see §1.1.** The
> recommendations below already reflect them.

---

## 0. The symptoms we are explaining

Observed on the real robot:

1. The planner "is not smart" — picks **convoluted / complex paths** when a
   simple one exists.
2. The robot **gets stuck a lot**, even when the task is easy.
3. It **does not use the omnisteering** well — it behaves like a
   differential-drive robot, doing awkward maneuvers.

The wheel-tracking / no-rotation issue (fixed by enlarging the **deadband**) is
**out of scope here** — noted as already resolved. This document is about
planner / controller / costmap / behavior tuning.

---

## 1. Robot facts that drive every recommendation

CONCERT is a **4-wheel omnidirectional (holonomic)** base. This is the single
most important fact, because almost every Nav2 default assumes differential
drive.

| Property | Value | Source |
|---|---|---|
| Drive type | Holonomic (x, y, θ independently) | base accepts `/omnisteering/cmd_vel` with `linear.y` |
| Footprint | `[[-0.685,-0.4],[0.685,0.4]]` → **1.37 m × 0.80 m** | `*_costmap.yaml` |
| Footprint padding | 0.03 m | `*_costmap.yaml` |
| **Inscribed radius** | **0.43 m** (with padding) | computed: `min(half_x, half_y)` |
| **Circumscribed radius** | **0.83 m** (with padding) | computed: `hypot(half_x, half_y)` |
| Sensors | front + back VLP16 → **360° coverage**, no privileged "front" | lidar fusion → `/scan` |
| Base cmd input | `/omnisteering/cmd_vel` (`geometry_msgs/Twist`) | `README.md` |

**Consequences:**
- The robot is large and *long* (1.37 m). Clearance matters a lot.
- It is holonomic — it *can* strafe and rotate independently, which is valuable
  for fine alignment and tight spaces (even if it generally faces forward).
- Any costmap inflation **smaller than the inscribed radius (0.43 m) is unsafe
  and under-represents collisions** (see §2).

### 1.1 Design decisions (locked 2026-06-05)

These four choices drive every recommendation below:

| # | Question | **Decision** | Drives |
|---|---|---|---|
| 1 | Map mode | **Saved static map** | keep `allow_unknown: false` (§5.1) |
| 2 | Narrow corridors? | **Yes — must traverse them** | **moderate**, not large, inflation (§2) |
| 3 | Planner | **Keep SmacPlanner2D** (my call — §5.4) | *not* Theta\*; Lattice only if ever needed |
| 4 | Path style | **Generally face travel direction** | forward-facing critics, keep Omni (§4) |

---

## 2. 🔴 P0 — Costmap inflation is smaller than the robot (root cause of "stuck" and "complex paths")

**This is the highest-impact finding.** Both costmaps currently use:

```yaml
inflation_layer:
  cost_scaling_factor: 3.0
  inflation_radius: 0.20      # global_costmap.yaml:43  and  local_costmap.yaml:19
```

`inflation_radius = 0.20 m` is **below the robot's inscribed radius of 0.43 m.**

### Why this breaks everything

Nav2's inflation layer does two things around every lethal obstacle
([Inflation Layer docs](https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html)):

1. Marks everything within the **inscribed radius** as lethal-inscribed (cost
   253) — "places a lethal cost around obstacles within the robot's fully
   inscribed radius, even if a robot is non-circular."
2. Beyond that, lays down an **exponential decay** potential out to
   `inflation_radius`:
   `cost(d) = 252 · exp(−cost_scaling_factor · (d − inscribed_radius))`.

When `inflation_radius (0.20) < inscribed_radius (0.43)`, the inflation loop
stops **before** it has even finished drawing the inscribed lethal ring. Nav2
logs this as an error in
[`inflation_layer.cpp`](https://api.nav2.org/nav2-humble/html/inflation__layer_8cpp_source.html)
(lines 173–181):

> *"The configured inflation radius (%.3f) is smaller than the computed
> inscribed radius (%.3f) of your footprint, it is highly recommended to set
> inflation radius to be at least as big as the inscribed radius to avoid
> collisions."*

Effects, which match the symptoms exactly:
- **SmacPlanner2D treats the robot as a circle** and checks only the cost at the
  robot *center* (holonomic planners do not use the SE2 footprint —
  [Tuning Guide → Footprint vs Radius](https://docs.nav2.org/tuning/index.html)).
  With almost no potential field, A* routes the center to within ~0.2 m of
  walls → the real **1.37 m × 0.80 m footprint clips the wall**.
- **MPPI's `CostCritic` (`consider_footprint: true`) then rejects** those
  wall-adjacent poses the plan told it to follow → the controller stalls,
  oscillates, the progress checker trips → recovery. → **"stuck."**
- With no smooth potential field, A* has nothing to "lean on," so paths hug
  corners and look **erratic / complex** instead of staying centered.

This is a **well-known, documented failure mode.** Nav2 GitHub issue
[#6100 "SMAC always generates a path that hugs the wall when turning, making it
impossible to move forward"](https://github.com/ros-navigation/navigation2/issues/6100)
is the identical symptom; the maintainer's fix was *"increase your inflation
radius."*

The [Nav2 Tuning Guide → Inflation Potential Fields](https://docs.nav2.org/tuning/index.html)
is explicit:

> *"The true value of the inflation layer is creating a consistent potential
> field around the entire map … increase your inflation layer cost scale and
> radius in order to adequately produce a smooth potential across the entire
> map."*

### Recommended change (starting point — tune on the robot)

| File | Param | Current | Proposed | Rationale |
|---|---|---|---|---|
| `local_costmap.yaml` | `inflation_radius` | `0.20` | **`0.50`** | just above inscribed `0.43` — safe, but **lean so it does not flood narrow corridors** with cost (MPPI reads this map) |
| `local_costmap.yaml` | `cost_scaling_factor` | `3.0` | **`3.0`** (keep) | steeper decay clears the corridor centre faster → no MPPI "creep / jitter" in tight spaces |
| `global_costmap.yaml` | `inflation_radius` | `0.20` | **`0.60`** | ≥ inscribed, a little more for centring on the static map; still corridor-safe |
| `global_costmap.yaml` | `cost_scaling_factor` | `3.0` | **`3.0`** (keep) | — |

**Why moderate, not large (decision #2 — narrow corridors):** the inflation
*must* be ≥ the inscribed radius `0.43 m` (safety, complete lethal ring), but we
deliberately keep it **modest** rather than the ~0.85 m a pure open-space setup
would use. A large radius floods a ~1 m corridor with cost end-to-end, which
makes MPPI **creep** and can make the global planner **detour around** a passage
you need. The centring you want in open areas comes mainly from the planner's
`cost_travel_multiplier` (§5.2), not from a huge inflation. **Tuning rule:**
never below `0.43`; if a real corridor is still refused, drop the **local** value
toward `0.45`; if open-space paths hug walls too much *and* corridors are never
a problem, raise both ~`0.15–0.25 m`.

> This supersedes/expands `DEBUG_FINDINGS.md` **A12** — the value is now `0.20`,
> i.e. *worse* than when A12 was written. The principled minimum is the
> inscribed radius `0.43`.

---

## 3. 🔴 P0 — Command chain is mis-wired: the velocity smoother is bypassed

Your [`README.md`](README.md) documents the **intended** command chain:

```text
controller/recoveries -> cmd_vel_nav
cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
cmd_vel_smoothed -> collision_monitor -> /omnisteering/cmd_vel
```

But the **actual** wiring does not match it.

- `velocity_smoother` subscribes to `cmd_vel` (remapped to `cmd_vel_nav`) and
  **publishes `cmd_vel_smoothed`** — confirmed in the Nav2 source
  ([`velocity_smoother.cpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_velocity_smoother/src/velocity_smoother.cpp):
  `smoothed_cmd_pub_ = ... TwistPublisher(node, "cmd_vel_smoothed")`).
- `collision_monitor.yaml:5` sets **`cmd_vel_in_topic: "cmd_vel_nav"`** — so the
  collision monitor reads the **raw controller output**, not the smoothed one.

**Result:** `velocity_smoother` runs but its `cmd_vel_smoothed` output is
consumed by **nobody**; `collision_monitor` takes the raw MPPI command straight
to the base. The smoother — including its `deadband_velocity: [0.05,0.05,0.05]`
and acceleration ramping — is **completely inert.**

### Recommended change

| File | Param | Current | Proposed |
|---|---|---|---|
| `collision_monitor.yaml` | `cmd_vel_in_topic` | `"cmd_vel_nav"` | **`"cmd_vel_smoothed"`** |

**Note for the deadband / rotation fix you already did:** if any part of that
fix was put in `velocity_smoother.yaml`, it is currently **not taking effect**
(bypassed). Verify whether the working deadband lives in MPPI's
`VelocityDeadbandCritic` (which *is* in the live chain) or in the base driver.
This is *not* "stuck" causing on its own, but it is a real correctness gap and
makes motion jerkier than designed.

---

## 4. 🟠 P0/P1 — MPPI controller: tune for clean *forward-facing* holonomic tracking

The controller is **MPPI with `motion_model: "Omni"`** — the correct choice
([MPPI docs](https://docs.nav2.org/configuration/packages/configuring-mppic.html):
works with Differential, Omnidirectional, Ackermann). Per **decision #4 you want
the robot to generally face its travel direction** — so we tune the *heading*
critics for forward-facing tracking, while **keeping the Omni model** so the
base can still strafe for fine corrections and tight-space maneuvering.

> Your earlier feeling that it "doesn't use the omnisteering" was mostly the
> inflation problem (§2) and the bypassed smoother (§3) making it stall and
> thrash — **not** a deliberate diff-vs-omni setting. Fix those and choose
> forward-facing tracking, and it will drive cleanly (facing where it goes) yet
> still slide sideways when that genuinely helps.

### 4.1 Make it face the path: `PathAngleCritic` mode `1` → `0`

```yaml
PathAngleCritic:            # controller.yaml:114
  mode: 1                   # 1 = NoDirectionalPreference  ->  CHANGE TO 0
```

Mode **0 (Forward Preference)** *"penalizes high path angles relative to the
robot's orientation to incentivize turning towards the path"*
([MPPI docs](https://docs.nav2.org/configuration/packages/configuring-mppic.html)) —
i.e. the robot rotates to **face its direction of travel**, exactly what you
asked for. Mode 1 (the current value) instead lets it travel backwards whenever
that is shorter, which is *not* "face travel direction."

### 4.2 Keep `PreferForwardCritic` enabled (it now *helps* you)

```yaml
PreferForwardCritic:        # controller.yaml:85
  enabled: true             # KEEP enabled
  cost_weight: 1.5
```

It *"incentivizes moving in the forward direction, rather than reversing"*
([MPPI docs](https://docs.nav2.org/configuration/packages/configuring-mppic.html)).
That reinforces forward-facing travel and pairs naturally with PathAngle mode 0.
If the robot still reverses more than you like, raise `cost_weight` toward ~3–5.

> **Note:** this reverses the suggestion in the first draft of this document,
> which assumed you wanted *free strafing*. Your decision #4 (face travel
> direction) means we **keep** the forward bias instead of removing it.

### 4.3 Keep these as-is

| Critic / param | Current | Keep? | Why |
|---|---|---|---|
| `TwirlingCritic` | weight `10.0` | ✅ keep | Penalizes *unnecessary spinning* — keeps forward tracking clean, no idle twirling. |
| `VelocityDeadbandCritic` | weight `35`, `[0.05,0.05,0.1]` | ✅ keep | Matches the base deadband; stops MPPI commanding velocities the base can't execute. |
| `motion_model` | `"Omni"` | ✅ **keep (do NOT switch to DiffDrive)** | Retains `vy` so the robot can strafe for fine alignment and to squeeze through narrow corridors, even though it generally faces forward. |
| `GoalAngleCritic` | weight `3.0` | ✅ keep | Aligns final yaw to the goal. |

### 4.4 Optional — `RotationShimController` for clean starts (now recommended)

Because you want forward-facing motion, wrapping MPPI in
`RotationShimController` is now a **good optional add**: it rotates the robot in
place to the new path's heading *before* tracking, avoiding the
"whipping / stuttering" start-up that holonomic controllers can show
([Tuning Guide → Rotate in Place](https://docs.nav2.org/tuning/index.html);
suited to "differential and omnidirectional robots"). Consider it if path-start
motion looks awkward after the P0 fixes.

### 4.5 Minor

- `PathAlignCritic` weight `10.0` (doc default 14). Higher = tighter path
  tracking; keep ~10–14, revisit only if motion is too rigid/loose.
- `AckermannConstraints.min_turning_r: 0.2` is **ignored** under the Omni model.
  Harmless; can delete for clarity.

---

## 5. 🟠 P1 — Global planner (SmacPlanner2D)

```yaml
GridBased:                         # planner_server.yaml
  plugin: "nav2_smac_planner::SmacPlanner2D"
  allow_unknown: false
  cost_travel_multiplier: 2.0
  tolerance: 0.5
```

SmacPlanner2D is an appropriate **holonomic** planner (8-connected A*, circular
assumption) — good for an omni robot because it imposes no turning-radius
constraint. We **keep it** (decision #3, §5.4). Three things to address:

### 5.1 `allow_unknown: false` → **keep `false`** (decision #1: saved map)

With `false`, the planner refuses to route through unknown cells. That is the
**correct, safe choice for the saved static map you are using** — it won't plan
into unmapped voids. (On an incomplete/live-SLAM map this would cause "no path"
stalls, which is why it was an open question — but with a complete saved map,
keep it `false`.) Revisit only if you later navigate while mapping.

### 5.2 `cost_travel_multiplier` → raise after the inflation fix

*"Larger values place [the path] in the center of aisles more exactly (if a
non-FREE cost potential field exists)"*
([Smac 2D docs](https://docs.nav2.org/configuration/packages/smac/configuring-smac-2d.html)).
This is the knob that makes A* **use** the potential field from §2 — and, with
the deliberately-moderate inflation we chose, it is now the **main** source of
path-centring. With a real potential field present, raise to **`3.0`** (up to
`3.5`). It does nothing while inflation is tiny — so apply §2 **first**.

### 5.3 `tolerance: 0.5` → `0.25`

Align planner tolerance with the goal checker's `xy_goal_tolerance: 0.25` so the
planner doesn't quietly stop 0.5 m short. Minor.

### 5.4 Planner choice (decision #3): **keep SmacPlanner2D — *not* Theta\***

You asked me to pick. For **a large (1.37 m) robot that must thread narrow
corridors on a saved map, SmacPlanner2D is the right choice** and Theta\* is
not:

- **SmacPlanner2D** is cost-aware A\*. With the §2 potential field and a raised
  `cost_travel_multiplier` it *"places the path in the center of aisles more
  exactly"*
  ([Smac 2D docs](https://docs.nav2.org/configuration/packages/smac/configuring-smac-2d.html))
  — exactly what narrow corridors need. Raw paths are 45°/90° staircases, but
  the planner's internal smoother + MPPI's local re-optimization clean that up,
  so the *executed* motion is smooth.
- **Theta\*** makes any-angle straight lines via line-of-sight, which **cut
  corners close to obstacles**. For a long robot in tight spaces that is risky —
  the path can pass nearer to walls than the robot's body can clear. So we avoid
  it here despite its "straighter-looking" paths.

**Known limitation (and the safety net):** every *holonomic* planner
(SmacPlanner2D, Theta\*, NavFn) approximates the robot as its **0.43 m inscribed
circle and ignores the 1.37 m length**
([Tuning Guide → Footprint vs Radius](https://docs.nav2.org/tuning/index.html)).
The robot's full rectangular footprint is enforced **at control time** by MPPI's
`CostCritic` (`consider_footprint: true`). This planner+controller division of
labor is fine for normal building corridors.

**Escalation path (don't do it yet):** if you hit genuinely tight spots where
the robot's *length* must be considered *during planning* (e.g. it must angle
diagonally through a gap), switch to **`SmacPlannerLattice`** — the planner the
Nav2 maintainers list for **non-circular omnidirectional** robots, which plans
with the full SE2 footprint and motion primitives. It is heavier and needs a
control set, so only escalate if SmacPlanner2D + MPPI demonstrably can't make a
specific passage.

---

## 6. 🟡 P2 — Path smoother server is disabled

The `SimpleSmoother` server is **commented out** in both the launch file
(`path_planner.launch.py:102-109`) and the BT (`behavior.xml:10-12`
`<SmoothPath>` is commented). SmacPlanner2D's **internal CG smoother** (the
`smoother:` block in `planner_server.yaml`) still runs, so paths get *some*
smoothing.

- **Staying on SmacPlanner2D (our choice):** leaving the server off is
  acceptable — MPPI re-optimizes locally anyway. Low priority.
- Re-enable the `smoother_server` node **and** the `<SmoothPath>` BT node only
  if global paths still look rough after §2/§5.

No change strictly required; documented for completeness.

---

## 7. 🟡 P2 — Recovery behaviors

```yaml
backup:  backup_dist=0.15, backup_speed=0.025   # behavior.xml RoundRobin
```

- The **BackUp** recovery is glacial: 0.15 m at 0.025 m/s = **6 s** to move
  15 cm — negligible for a 1.37 m robot. Consider `backup_dist ≈ 0.30`,
  `backup_speed ≈ 0.08–0.10`.
- `Spin 1.57` and `Wait 5` are fine.
- Recoveries only fire **after** the robot is already stuck — fixing §2/§4/§5
  should make them rare. Tune these last.

`recovery.yaml` `max_rotational_vel: 0.4`, `rotational_acc_lim: 0.6` are
sensibly reduced for a large robot. Keep.

---

## 8. 🟡 P2 — Behavior tree & checkers

- **Active BT** = `behavior.xml` (per `path_planner.launch.py:28`).
  `basic.xml` is unused (no recovery branch) — keep as a reference or delete.
- `RateController hz="1.0"` → global replans at **1 Hz**. Fine for a slow base;
  raise to `2.0` if the robot reacts sluggishly when newly blocked.
- `progress_checker`: `required_movement_radius: 0.5`, `movement_time_allowance:
  10.0` → if MPPI makes < 0.5 m progress in 10 s, recovery triggers. Reasonable;
  if recoveries fire too eagerly once root causes are fixed, relax the time
  allowance. Don't relax it *before* fixing §2 (that would just hide stalls).
- `general_goal_checker`: `xy 0.25 / yaw 0.25` — fine.

---

## 9. 🟡 P2 — Collision monitor cleanup (dead config)

```yaml
scan:                  # collision_monitor.yaml:42-47
  type: "scan"
  min_height: 0.15     # <-- IGNORED
  max_height: 2.0      # <-- IGNORED
```

The `"scan"` observation source has **no height filtering** — `min_height` /
`max_height` exist only on the `"pointcloud"` source type
(`nav2_collision_monitor/pointcloud.hpp`; absent from `scan.hpp`). These two
lines do nothing. (This is `DEBUG_FINDINGS.md` **A11**.) Remove them to avoid
implying a filter that isn't applied.

Also note (no change, just awareness): the collision monitor consumes the **raw
`/scan`** and its `PolygonStop` (1.5 × 1.0 m) is only slightly larger than the
footprint (1.43 × 0.86 m). After the §3 wiring fix it will consume
`cmd_vel_smoothed`. Watch for spurious stops if the merged scan has near-range
noise.

---

## 10. Prioritized plan (decisions locked 2026-06-05)

**Decisions applied to this plan:** (1) **saved static map** → keep
`allow_unknown: false`; (2) **must traverse narrow corridors** → moderate, not
large, inflation; (3) **keep SmacPlanner2D** (not Theta\*); (4) **generally face
travel direction** → forward-facing critics.

**Apply in this order** (each step's effect is largest once the previous is in
place). *Nothing here is applied yet — configs unchanged.*

| Prio | Change | File(s) | Expected effect |
|---|---|---|---|
| **P0-1** | Inflation ≥ inscribed but **moderate**: local `0.50` / global `0.60`, keep `cost_scaling 3.0` | `local_costmap.yaml`, `global_costmap.yaml` | Kills wall-hugging / stuck / erratic paths — *and* stays corridor-friendly |
| **P0-2** | `cmd_vel_in_topic: "cmd_vel_smoothed"` | `collision_monitor.yaml` | Restores the designed smoothing chain |
| **P0-3** | `PathAngleCritic.mode: 0` (keep `PreferForwardCritic`, keep `motion_model: Omni`) | `controller.yaml` | Robot **faces its travel direction** cleanly |
| **P1-4** | `cost_travel_multiplier: 3.0`, `tolerance: 0.25` | `planner_server.yaml` | Central, direct paths in corridors (needs P0-1 first) |
| **P1-5** | Keep `allow_unknown: false` | `planner_server.yaml` | *(decided — no change)* |
| **P2-6** | Remove dead `min/max_height` lines | `collision_monitor.yaml` | Clarity (A11) |
| **P2-7** | Faster BackUp recovery | `recovery.yaml` / BT | Better recovery (rarely hit) |
| **P2-8** | *(optional)* `RotationShimController` | `controller.yaml` / launch | Cleaner forward-facing path starts |

**Escalation (only if needed):** if SmacPlanner2D + MPPI can't make a specific
tight passage, switch the planner to `SmacPlannerLattice` (§5.4).

---

## 11. How we'll validate each change (when we apply them)

All checks below are read-only / no-goal except the last. Bring up the stack
with real localization on the saved map; send a goal **only** in open space,
near, slow, with the e-stop ready.

- **Inflation error gone:** after rebuild, costmap logs no longer print the
  "inflation radius smaller than inscribed" error.
- **Potential field present:** in RViz, the global costmap shows a *gradient*
  around walls (not a thin hard edge); the global path stays **centered** in
  free space instead of hugging walls; narrow corridors still show a
  low-cost central channel (not flooded solid).
- **Smoothing chain live:** `ros2 topic info /cmd_vel_smoothed` shows the
  collision monitor as a subscriber; `ros2 topic hz /cmd_vel_smoothed` matches
  control rate.
- **Faces travel direction:** with a goal ahead, the robot rotates to face the
  path then drives forward (no backward travel, no idle twirling); in tight
  alignment it may briefly strafe (`linear.y ≠ 0` on `/cmd_vel_nav`).
- **Fewer recoveries / no stalls:** the BT does not fall into the recovery
  `RoundRobin` on simple goals; progress checker doesn't trip.

---

## 12. Sources

- Nav2 MPPI Controller — config & critics:
  https://docs.nav2.org/configuration/packages/configuring-mppic.html
- Nav2 MPPI README (critic tables, motion models):
  https://github.com/ros-navigation/navigation2/blob/main/nav2_mppi_controller/README.md
- Nav2 Tuning Guide (inflation potential fields, footprint vs radius, planner /
  controller selection, rotate-in-place):
  https://docs.nav2.org/tuning/index.html
- Nav2 Inflation Layer params:
  https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html
- Inflation source — "inflation radius < inscribed" error
  (`inflation_layer.cpp`):
  https://api.nav2.org/nav2-humble/html/inflation__layer_8cpp_source.html
- Smac 2D Planner params (`cost_travel_multiplier`, `allow_unknown`):
  https://docs.nav2.org/configuration/packages/smac/configuring-smac-2d.html
- Issue #6100 — SMAC hugs wall when turning, fixed by larger inflation:
  https://github.com/ros-navigation/navigation2/issues/6100
- Issue #5021 — Omnidirectional robot MPPI settings:
  https://github.com/ros-navigation/navigation2/issues/5021
- PathAngleCritic source (mode / reversing fallback):
  https://api.nav2.org/nav2-jazzy/html/path__angle__critic_8cpp_source.html
- Velocity smoother source (publishes `cmd_vel_smoothed`):
  https://github.com/ros-navigation/navigation2/blob/main/nav2_velocity_smoother/src/velocity_smoother.cpp
- Velocity smoother params:
  https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html
