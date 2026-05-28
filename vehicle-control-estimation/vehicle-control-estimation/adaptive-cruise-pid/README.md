# Adaptive Cruise Control (PID)

> Follows a lead vehicle at a constant time-gap using a PID controller with integral anti-windup, acceleration limits, and a jerk constraint; falls back to set-speed control.

![demo](demo.png)

## How it works
- Desired gap `= tau*v_ego + standoff`; PID acts on gap error
- Min-select between gap-following and set-speed acceleration
- Integral clamping anti-windup + jerk-rate limiting for comfort

## Run it
```matlab
>> acc_pid        % prints metrics and saves demo.png
```

## Result
Maintains a safe gap (min ~22 m) through a lead-vehicle slow-down and speed-up event with no collision.

## What this demonstrates
Classical feedback control, controller saturation handling, and ADAS longitudinal control logic.

**Stack:** MATLAB (base install - no toolboxes required). Validated in GNU Octave.

---
*Part of the **Vehicle Control & Estimation** MATLAB experiments. MIT licensed.*
