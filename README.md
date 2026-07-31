# CamosunROV Testing Bar30 Pressure using Polling

- Product link: https://bluerobotics.com/store/sensors-cameras/sensors/bar-depth-pressure-sensor/
- Original Code: https://github.com/bluerobotics/BlueRobotics_MS5837_Library/tree/master
- Target: STM32F446RE (STM32CubeIDE / STM32CubeMX project)
- Sensor: Blue Robotics Bar30 (MS5837-30BA), polled over I2C

## Overview

This is a C port of the Blue Robotics MS5837 library for use on the STM32F446RE, polling the Bar30 pressure/depth sensor over I2C. The driver stores its configuration in a `device`/`dev` struct rather than class members, so values are set with `dev->` field access or setter functions depending on how the driver is wrapped.

## Default Configuration

When the driver is initialized, the following defaults are applied:

| Field | Default value | Meaning |
|---|---|---|
| `dev->model` | `MS5837_MODEL_30BA` | Sensor variant in use (Bar30). Only change this if you swap to a Bar02 (`MS5837_MODEL_02BA`). |
| `dev->surface_pressure_mbar` | `1013.25f` mbar | Reference atmospheric pressure at the surface (standard atmosphere), used to zero the depth calculation. 1 mbar = 1 hPa |
| `dev->fluidDensity_kg_m3` | `1026` kg/m³ | Density of the working fluid. Default is average ocean (salt) water. |
| `dev->secondOrderCalculation` | `false` | Uses first-order compensation only. Second-order compensation is more accurate at temperature extremes but costs more compute time. Used for water below 15C |

All units are SI (mbar for pressure, kg/m³ for density, °C for temperature, m for depth).

## Changing the Defaults

### 1. Sensor model — `SetModel()`
Only needed if you are not using a Bar30. Call before `Init()`/`begin()`:

```c
MS5837_SetModel(&dev, MS5837_MODEL_02BA); // switch to Bar02 instead of Bar30
```

### 2. Surface pressure reference — `SetSurfaceReference()`
Use this if you're testing somewhere other than sea level or want to zero the sensor against a live barometer reading instead of the standard atmosphere constant:

```c
MS5837_SetSurfaceReference(&dev, 1005.0f); // e.g. today's local barometric pressure in mbar
```

### 3. Fluid density — `setFluidDensity()`
Change this if testing in fresh water instead of salt water, since it directly affects the pressure-to-depth conversion:

```c
MS5837_SetFluidDensity(&dev, 997.0f);  // freshwater (e.g. pool testing)
MS5837_SetFluidDensity(&dev, 1026.0f); // saltwater / ocean (default)
```

### 4. Second-order temperature compensation — `secondOrderCalculation`
Enable this from `main.c` after init if you need higher accuracy at temperature extremes (near 0 °C or above 20 °C) and can afford the extra CPU cycles per read:

```c
dev.secondOrderCalculation = true;
```

Leave this `false` for routine pool/tank testing at room temperature, where first-order compensation is accurate enough and faster to compute — relevant when polling at a high rate on the STM32F446RE.
