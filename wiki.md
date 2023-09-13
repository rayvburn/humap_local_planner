## Parameters

### Trajectory sampling parameters

Trajectory samples used for search can be manipulated in many ways. Input data to samples selection are:
- minimum value,
- maximum value,
- granularity.

If one wants to check amplifiers from 0.1 to 10.0 that are located each 0.25, one must set:
- minimum value to `0.1`,
- maximum value to `10.0`,
- granularity to `0.25`.

On the other hand, to fix one of the amplifiers at a specific value, one must set:
- minimum value to `<FIXED VALUE>`,
- maximum value to `<FIXED VALUE>`,
- granularity to `>0` value.

The last special case is that one wants to discard values scaled by the multiplier, then set:
- minimum value to `<SOME VALUE>`,
- maximum value to `<SOME VALUE>`,
- granularity to `0` value.

If granularity exceeds min-max range, then only min and max values will be taken to sampling.

The easiest way to manipulate trajectory search parameters is to use Dynamic Reconfigure. Run:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

and switch to `Trajectory` tab.

### Trajectory generation parameters
If one has changed simulation time, planner frequency or so, he must also adjust basic force factors as time delta changes will affect force strengths.

### Tuning guide

#### `force_internal_amplifier`
A good practice is to debug all explored trajectories in `HumapPlanner::findBestTrajectory`. For a custom application, user should uncomment/investigate explored trajectories with respective velocity limits. Once translational velocity starts repeating among consecutive `force_internal_amplifiers`, one can trim `max` value to the highest that impacts overall velocity. Note that it is highly related to `internal_force_factor` itself. It's wise to tune `internal_force_factor` without planning first - adjust this parameter to satisfying robot speeds.

#### `sfm_aw_amplifier`
Values outside of range defined by `<-1.0; +1.0>` do not affect generated trajectories set at all.

#### Trajectory cost function parameters

- `goal_distance_scale`
  - bigger value implies lower costs along global path but in areas close to the actual goal
  - if robot tends to select slow trajectories with minimal rotations, try to increase this value
- `occdist_scale`
  - if too high (e.g., `1.05`), then robot may oscillate in proximity to goal
- `goal_front_scale`
  - keeping too low may instruct trajectory scorer to favour static trajectories instead of those going towards goal (when available)
  - keeping too high may favour trajectories that do not head towards goal in close proximity to the goal
- `backward_penalty`
  - must be evaluated experimentally - let robot go until it starts backing up (which does not make sense in that situation), stop the robot and tune the value until proper trajectory is selected
  - if `backward_scale` is zeroed, then `backward_penalty` does not affect scoring at all

#### Equisampled velocities trajectory generator parameters

The planner uses 2 trajectory generators to create trajectories that are later scored. `equisampled_min_vel_x` should be kept relatively small - approx. `0.1 m/s`. Setting `0.0 m/s` may produce in-place trajectories because those may be scored highest. On the other hand, in some cluttered areas setting this value too high, e.g., `0.3 m/s` will make the planner to not make use of the fallback-generator ("equisampled velocities trajectory generator") since those trajectories may be scored lower than, e.g., going to the goal using backwards motion.

## Usage

### Visual Studio Code

If one encounters ROS headers not being recognized by VSCode despite proper paths in `cpp_properties.json`, try to source workspace and then run VSCode from terminal:

```bash
cd <PATH_TO_YOUR_PROJECT>
source ../../devel/setup.bash
code .
```

No addons required then.

## Troubleshooting

### `Off Map`

Once you are getting `Off Map` warning messages try to reduce `forward_point_distance` parameter (`Dynamic Reconfigure` -> `Costs` tab) or increase local costmap size. Warnings look like this:

```console
[ WARN] [1648585717.627180821, 106.518000000]: Off Map 2.008240, 0.714039;
```

## Discontinued branches

- [`planning/feature-slow-translation-costfun`](https://github.com/rayvburn/humap_local_planner/tree/planning/feature-slow-translation-costfun) - implements a new cost function that penalizes robot for slow translation movements (this can also be achieved by a more in-depth tuning of other cost functions);
- [`planning/feature-visualization-map-grid-cost-functions`](https://github.com/rayvburn/humap_local_planner/tree/planning/feature-visualization-map-grid-cost-functions) - planner class exposes `getPathGoalFrontCosts()` and `getPathAlignmentCosts()` methods that use `base_local_planner::MapGridCostFunction::getTargetPoses` that is not available in the upstream repository. To use that feature, one may implement the following:

  ```diff
  diff --git a/base_local_planner/include/base_local_planner/map_grid_cost_function.h b/base_local_planner/include/base_local_planner/map_grid_cost_function.h
  index 421bd005..9196a06f 100644
  --- a/base_local_planner/include/base_local_planner/map_grid_cost_function.h
  +++ b/base_local_planner/include/base_local_planner/map_grid_cost_function.h
  @@ -118,6 +118,10 @@ public:
     // used for easier debugging
     double getCellCosts(unsigned int cx, unsigned int cy);

  +  std::vector<geometry_msgs::PoseStamped> getTargetPoses() const {
  +    return target_poses_;
  +  }
  +
   private:
     std::vector<geometry_msgs::PoseStamped> target_poses_;
     costmap_2d::Costmap2D* costmap_;
  ```

  See [`planner`: extended ROS visualization (pruned global plan, paths regarded in cost functions)](https://github.com/rayvburn/humap_local_planner/commit/05d26ab131e5de7898658f7b8f4a205d4e9194a0) and [`planner`: extended ROS visualization (pruned global plan)](https://github.com/rayvburn/humap_local_planner/commit/3a7520c8ee1422c41ecf66a35d665a5b42f422f4) diff for details.
