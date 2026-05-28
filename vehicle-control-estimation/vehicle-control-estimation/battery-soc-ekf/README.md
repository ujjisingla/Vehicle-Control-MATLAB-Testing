# Battery SOC Estimation (Extended Kalman Filter)

> Estimates lithium-ion State of Charge from noisy voltage/current using a 1st-order RC equivalent-circuit model and an EKF - a core Battery Management System competency.

![demo](demo.png)

## How it works
- State `x = [SOC; V_rc]`; measurement `y = OCV(SOC) - I*R0 - V_rc`
- Coulomb-counting prediction + RC relaxation dynamics
- EKF correction using the Jacobian of the nonlinear OCV(SOC) curve

## Run it
```matlab
>> soc_ekf        % prints metrics and saves demo.png
```

## Result
Converges from a deliberately wrong 60% initial guess to true SOC with **0.14% RMSE** under 20 mV measurement noise.

## What this demonstrates
State estimation, Kalman filtering, and equivalent-circuit battery modeling - directly relevant to BMS roles at Tesla, Rivian, and Lucid.

**Stack:** MATLAB (base install - no toolboxes required). Validated in GNU Octave.

---
*Part of the **Vehicle Control & Estimation** MATLAB experiments. MIT licensed.*
