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
A good practice is to debug all explored trajectories in `HuberoPlanner::findBestTrajectory`. For a custom application, user should uncomment/investigate explored trajectories with respective velocity limits. Once translational velocity starts repeating among consecutive `force_internal_amplifiers`, one can trim `max` value to the highest that impacts overall velocity. Note that it is highly related to `internal_force_factor` itself. It's wise to tune `internal_force_factor` without planning first - adjust this parameter to satisfying robot speeds.

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
