# Go2 Navigation Change Log

This file records navigation and localization changes so tuning decisions can be traced and reverted.

## 2026-07-15 - Uncommitted Nav2 local controller tuning

Status: uncommitted

Purpose:
- Improve behavior when a dynamic obstacle appears in front of the robot and the global planner has already found a path, but the local controller stops instead of moving.

Changes:
- Increased local costmap window from `5 x 5 m` to `6 x 6 m`.
- Reduced local dynamic obstacle persistence from `1.5 s` to `0.5 s`.
- Reduced local inflation radius from `0.70 m` to `0.55 m`.
- Increased lateral MPPI sampling and lateral velocity allowance:
  - `vy_std: 0.05 -> 0.10`
  - `vy_max: 0.20 -> 0.30`
- A later test showed the remaining weak lateral motion was an execution
  deadband in the Go2 bridge, not a Nav2 lateral planning limit. The extra Nav2
  lateral increase was reverted and handled in `go2_bridge` instead.
- Slightly increased angular sampling:
  - `wz_std: 0.35 -> 0.45`
- Reduced path-following stiffness so MPPI can locally deviate around obstacles:
  - `PreferForwardCritic.cost_weight: 5.0 -> 2.5`
  - `PathAlignCritic.cost_weight: 4.0 -> 2.5`
  - `PathAngleCritic.cost_weight: 2.0 -> 1.2`
- Kept hard collision protection:
  - `collision_cost: 1000000.0`

Files:
- `nav2_config/nav2_params.yaml`

Validation:
- YAML parse check passed.
- No rebuild required; restart Nav2 to apply.

## 2026-07-15 - Uncommitted Go2 bridge low-speed deadband

Status: uncommitted

Superseded:
- This bridge-local deadband approach was replaced by the `go2_cmd_adapter`
  entry below. The bridge now only clamps maximum speed and converts Twist to
  Go2 sport API calls.

Purpose:
- Avoid Go2 jitter when Nav2 publishes small non-zero `/cmd_vel` values that
  are below the robot's reliable walking threshold.
- Observed example:
  - `linear.x ~= 0.14-0.15 m/s`
  - `linear.y ~= 0.05-0.06 m/s`
  - `angular.z ~= 0.0 rad/s`

Changes:
- Removed minimum-speed boosting from `go2_bridge`.
- If the commanded linear velocity vector is below `lin_vel_deadband`, it is
  sent as zero instead of being amplified.
- Commands above the deadband are passed through unchanged, preserving Nav2's
  planned x/y direction and magnitude.
- Small angular commands below `ang_vel_deadband` are sent as zero.
- Added parameters:
  - `lin_vel_deadband: 0.15`
  - `ang_vel_deadband: 0.05`
- After testing, the linear deadband was reduced from `0.18` to `0.15` to avoid
  filtering out small but useful Nav2 translation commands.
- Reason for change:
  - Minimum-speed boosting helped with startup but could amplify tiny controller
    corrections during turns, causing jitter or unwanted lateral motion. The
    safer behavior is to stop on too-small commands and solve execution tracking
    later with a proper closed-loop adapter.

Files:
- `src/go2_bridge/go2_bridge/bridge.py`

Validation:
- `python3 -m py_compile src/go2_bridge/go2_bridge/bridge.py`
- `colcon build --packages-select go2_bridge --symlink-install --event-handlers console_direct+`

## 2026-07-15 - Uncommitted Go2 cmd_vel adapter

Status: uncommitted

Purpose:
- Move Go2-specific command shaping out of `go2_bridge` so Nav2 planning is not
  polluted and bridge code only converts Twist commands to Go2 sport API calls.
- Avoid simple per-axis lateral boosting, which changed Nav2's intended motion
  direction and caused odd movement.

Changes:
- Added `go2_cmd_adapter`:
  - input: `/cmd_vel`
  - output: `/cmd_vel_go2`
- Changed `go2_bridge` default input to `/cmd_vel_go2`.
- Updated `start_navigation.sh` to launch adapter before bridge.
- Simplified `go2_bridge` to only clamp maximum speed and publish Go2 MOVE
  requests.

Adapter rules:
- If the command looks like in-place rotation:
  - `abs(wz) >= 0.25`
  - `hypot(vx, vy) < 0.16`
  - then clear `vx/vy` and keep `wz`.
- If planar speed is below `0.08`, clear `vx/vy`.
- If planar speed is below `0.22`, boost the x/y vector without changing its
  direction, limited by `max_boost_ratio: 2.2`.
- Limit command acceleration:
  - planar axes: `0.8 m/s^2`
  - yaw: `1.5 rad/s^2`

Files:
- `src/go2_bridge/go2_bridge/cmd_adapter.py`
- `src/go2_bridge/go2_bridge/bridge.py`
- `src/go2_bridge/setup.py`
- `start_navigation.sh`

Validation:
- `python3 -m py_compile src/go2_bridge/go2_bridge/bridge.py src/go2_bridge/go2_bridge/cmd_adapter.py`
- `python3 -c "import yaml; yaml.safe_load(open('nav2_config/nav2_params.yaml')); print('yaml ok')"`
- `colcon build --packages-select go2_bridge --symlink-install --event-handlers console_direct+`

## 2026-07-15 - Uncommitted obstacle clearance tuning

Status: uncommitted

Purpose:
- Reduce paths that hug obstacle edges and create tight arc-following behavior.
- Make global planning prefer the center of safer low-cost space instead of
  cutting along inflated obstacle boundaries.
- Reduce local controller hesitation caused by tracking a path that is already
  too close to obstacles.

Changes:
- Increased MPPI obstacle cost weight:
  - `CostCritic.cost_weight: 24.0 -> 32.0`
- Widened local costmap inflation and made its cost decay slower:
  - initially tested `local inflation_radius: 0.55 -> 0.65`
  - adjusted to `local inflation_radius: 0.58`
  - adjusted to `local cost_scaling_factor: 3.2`
- Widened global costmap inflation more strongly so the global planner plans
  farther away from obstacles:
  - initially tested `global inflation_radius: 0.70 -> 0.95`
  - adjusted to `global inflation_radius: 0.78`
  - reduced to `global inflation_radius: 0.62` after large-radius detours were observed
  - adjusted to `global cost_scaling_factor: 3.2`
- Made SmacPlanner2D more cost-aware:
  - initially tested `cost_travel_multiplier: 2.0 -> 4.0`
  - adjusted to `cost_travel_multiplier: 2.8`
  - reduced to `cost_travel_multiplier: 1.8` to avoid large global detour arcs
- Reason for adjustment:
  - The stronger inflation/cost settings made the global planner route around
    obstacles with too large an arc. The current values make the global path
    shorter and more direct, leaving close-range safety to the local costmap and
    MPPI controller.

Files:
- `nav2_config/nav2_params.yaml`

Validation:
- `python3 -c "import yaml; yaml.safe_load(open('nav2_config/nav2_params.yaml')); print('yaml ok')"`

## 2026-07-15 - Stabilize ICP localization and dynamic obstacle planning

Commit: `6af5b9f Stabilize ICP localization and dynamic obstacle planning`
Status: committed and pushed to `origin/master`

Purpose:
- Make dynamic obstacles visible to global planning.
- Smooth Go2 navigation behavior.
- Prevent ICP false convergence from publishing large TF jumps.

Changes:
- Added `/scan_leveled` obstacle input to `global_costmap`.
- Reduced Nav2 controller load and command aggressiveness:
  - Lowered max forward, lateral, and angular velocity.
  - Lowered angular acceleration.
  - Disabled MPPI noise regeneration to reduce command jitter.
- Added ICP gating in `fast_icp_loc`:
  - Rejects suspicious ICP updates by translation delta, yaw delta, and fitness score.
  - Keeps publishing previous pose/TF when an ICP update is rejected.

Files:
- `nav2_config/nav2_params.yaml`
- `src/fast_icp_loc/config/fast_icp_loc.yaml`
- `src/fast_icp_loc/include/fast_icp_loc/fast_icp_loc.hpp`
- `src/fast_icp_loc/src/fast_icp_loc.cpp`

Validation:
- `python3 -c "import yaml; yaml.safe_load(open('nav2_config/nav2_params.yaml')); print('yaml ok')"`
- `colcon build --packages-select fast_icp_loc --symlink-install --event-handlers console_direct+`

## 2026-07-14 - Nav2 behavior and RViz map QoS checkpoint

Commit: `a849b9b Tune Nav2 behavior and RViz map QoS`
Status: committed

Purpose:
- Save a known checkpoint before later localization and navigation experiments.

Changes:
- Tuned Nav2 motion limits and obstacle handling.
- Adjusted local/global costmap inflation and obstacle height parameters.
- Added path smoothing behavior tree:
  - `nav2_config/navigate_to_pose_w_smoothing.xml`
- Updated RViz map/costmap QoS for more reliable map display.

Files:
- `nav2_config/nav2.rviz`
- `nav2_config/nav2_params.yaml`
- `nav2_config/navigate_to_pose_w_smoothing.xml`

Notes:
- This was the checkpoint used when later experimental changes were reverted.
