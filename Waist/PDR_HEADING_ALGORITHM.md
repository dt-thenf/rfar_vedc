# Heading and PDR XY algorithm notes

This document describes the limited heading/XY refactor in `Waist/`. Activity recognition, step detection, BLE framing, and the helmet-side receiver remain unchanged.

## Sensor frame and mounting

The body frame used by the heading algorithm is:

- `+X`: up toward the sky
- `+Y`: to the wearer's left
- `-Z`: forward, in the direction whose bearing is estimated

The identity axis maps in `Compass_Module.cpp` assume ADXL345, ITG-3200, and HMC5883L axes are physically aligned with one another and mounted in this frame. Verify that assumption against the actual board layout; if the sensor packages are rotated relative to one another, apply a separate signed axis permutation for each sensor.

The gravity estimator assumes the accelerometer reports specific force with approximately +1 g along the upward axis at rest. The project passes ADXL345 values to step detection as if they are in g. Confirm this against the exact ADXL345 library in the build environment.

## Tilt-compensated heading

1. Propagate the estimated upward vector in body coordinates using the gyroscope angular rate.
2. Correct the estimate slowly toward the normalized accelerometer vector only when acceleration magnitude is plausibly near 1 g. This complementary correction avoids treating large gait/impact accelerations as pure gravity.
3. Apply magnetometer hard-iron offsets and per-axis soft-iron scale factors.
4. Remove the magnetic field component parallel to the estimated vertical vector. The remainder is the horizontal magnetic-north direction.
5. Project body-forward (-Z) into the horizontal plane and compute the signed angle from north to forward. The output convention is clockwise bearing: 0 degrees north, 90 degrees east, 180 degrees south, 270 degrees west.
6. Apply circular low-pass filtering and reject degenerate/implausible magnetic samples.

The hard-iron offsets, soft-iron scales, and magnetic declination are neutral defaults in source. They must be measured/configured for the final assembled device. A per-axis scale correction is only an approximation; use an ellipsoid-fit calibration when cross-axis soft-iron distortion is significant. The output currently uses magnetic north unless `MAGNETIC_DECLINATION_DEG` is configured for true north.

## PDR XY integration

The existing step counter and distance estimate remain the source of step events and distance. Positioning now integrates each new distance increment with the latest valid filtered heading, rather than locking one heading for an entire walking segment:

```
dX = distance * sin(heading)
dY = distance * cos(heading)
```

Map X increases east and map Y increases north. If a step/distance update arrives while heading is temporarily invalid, the counters remain pending and are consumed when a valid heading returns. Coordinates continue to expose both the raw estimate and bounded values within the existing 100 m x 50 m map. This is still step-and-heading PDR and will drift without external position fixes or map constraints.

The waist-to-helmet BLE call also now passes `getPositionX()` for posX and `getPositionY()` for posY.

## Bench checks before field use

- With the device level and pointed to known cardinal directions, check 0/90/180/270-degree convention and any fixed mounting offset.
- Rotate the device in roll and pitch while keeping its forward direction fixed; heading should remain approximately constant.
- Rotate through north (359 -> 0 -> 1 degrees); the filter should take the short angular path.
- Collect magnetometer samples through a full 3D rotation with the final enclosure and nearby components fitted. Fit hard-iron offsets and soft-iron correction from that data.
- Walk straight, then walk a right-angle turn; verify both X and Y change in the expected direction.
- Validate that the BLE receiver reports distinct X and Y values. This change fixes the previous duplicate Y argument but does not change the BLE packet format.

## References

1. NXP/Freescale, [AN4248: Implementing a Tilt-Compensated eCompass using Accelerometer and Magnetometer Sensors](https://www.nxp.com/docs/en/application-note/AN4248.pdf) — tilt compensation, coordinate alignment, and heading computation.
2. NXP/Freescale, [AN4246: Calibrating an eCompass in the Presence of Hard- and Soft-Iron Interference](https://www.nxp.com/docs/en/application-note/AN4246.pdf) — hard-/soft-iron calibration.
3. Robert Harle, [A Survey of Indoor Inertial Positioning Systems for Pedestrians](https://doi.org/10.1109/SURV.2012.121912.00075), IEEE Communications Surveys & Tutorials, 2013 — step detection, heading systems, and drift limitations.
4. Yuan Wu et al., [A Survey of the Research Status of Pedestrian Dead Reckoning Systems Based on Inertial Sensors](https://doi.org/10.1007/s11633-018-1150-y), International Journal of Automation and Computing, 2019 — PDR architectures and inertial sensor fusion.
5. [ADXL345 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf), [ITG-3200 product specification](https://invensense.tdk.com/wp-content/uploads/2015/02/ITG-3200-Datasheet.pdf), and [HMC5883L datasheet](https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf) — sensor behavior and output characteristics.

## Limitations

No physical sensor trace or hardware-in-the-loop test is available in this repository operation. Heading accuracy depends on sensor-axis alignment, magnetometer calibration, local magnetic interference, gyro bias, and the assumed accelerometer units/sign. PDR remains a relative estimate; it cannot guarantee long-term absolute XY accuracy without periodic external anchors.
