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
