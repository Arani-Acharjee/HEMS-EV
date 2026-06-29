# ⚡ Hybrid Energy Management System (HEMS) for Electric Vehicles

> A rule-based Energy Management Strategy implemented in MATLAB/Simulink, using **Stateflow** for control logic and **Simscape Electrical** for physical power-network simulation.

**Tech Stack:** MATLAB · Simulink · Simscape Electrical · Stateflow  
**Model File:** `HEMS_in_EV_Model.slx`  
**License:** MIT

---

## Overview

This project models a **Hybrid Energy Management System (HEMS)** for an electric vehicle that intelligently splits power between a **battery** and an **engine** based on the battery's State of Charge (SOC) and the instantaneous load demand.

The controller uses a **three-state Stateflow chart** to decide the operating mode at every instant, with smooth power transitions to avoid abrupt switching — a known weakness of simple threshold-only rule controllers.

The electrical network is implemented using **Simscape Electrical** components (controlled current sources, voltage/current sensors, resistor), bridged to the Simulink control logic via PS-Simulink and Simulink-PS converters.

---

## System Architecture

The model consists of four functional groups:

**1. Load Profile Source** — A `Repeating Sequence` block generates a time-varying load demand `P_load` that cycles through a 5-point table every 20 seconds.

**2. Stateflow EMS Controller** — Receives `P_load` and the current `SOC`, and outputs `P_battery` and `P_engine` based on whichever of the three states (Battery / Hybrid / Engine) is currently active.

**3. Simscape Electrical Network** — Battery and Engine are modeled as controlled current sources. A 1 Ω resistor acts as the load. Voltage and current sensors measure the resulting electrical quantities.

**4. SOC Feedback Path** — Measured battery current is scaled (Gain = 1/3600), negated, integrated (IC = 80%), clamped (0–100%), and smoothed (Low-Pass Filter, τ = 5) to produce the SOC signal fed back into the Stateflow chart.

```
Repeating Sequence (P_load)
        │
        ▼
Stateflow Chart ──── P_battery ──▶ Battery (Controlled Current Source)
  Battery / Hybrid /                        │
  Engine states     ── P_engine ──▶ Engine  │
        ▲                          (CCS)    │
        │ SOC                               ▼
        │                         Resistor (1 Ω) + Sensors
        │                                   │
        └── LPF ← Saturation ← Integrator ◀─┘
                                (IC = 80%)
```

---

## Energy Management Strategy

The Stateflow chart implements three operating modes, switched based on SOC thresholds and load magnitude.

### State: Battery (default)

The battery alone supplies the load up to a maximum of 1500 W. Any excess beyond that cap is passed to the engine. This is the initial state at simulation start (SOC = 80%).

```matlab
P_batt_max = 1500;
if P_load <= P_batt_max
    P_battery = P_load;
    P_engine  = 0;
else
    P_battery = P_batt_max;
    P_engine  = P_load - P_battery;
end
```

Transitions out when: `SOC < 60 && P_load > 1500 && after(1, sec)` → **Hybrid**

---

### State: Hybrid

Battery power is smoothly blended toward a SOC-proportional target using a first-order filter (smoothing factor `k = 0.2`). The engine covers the remainder.

```matlab
P_batt_max    = 1500;
k             = 0.2;
P_batt_target = min(P_batt_max * (SOC/100), P_load);
P_battery     = (1 - k) * P_battery + k * P_batt_target;
P_engine      = max(0, P_load - P_battery);
```

Transitions out when:
- `SOC > 65 && after(1, sec)` → **Battery**
- `SOC <= 30 && after(1, sec)` → **Engine**

---

### State: Engine

Same smoothing logic as Hybrid, but entered only at low SOC. Battery assist is reduced proportionally to the remaining charge; the engine carries the bulk of the load.

Transitions out when: `SOC > 35 && after(1, sec)` → **Hybrid**

---

### Transition Summary

| From | To | Condition |
|---|---|---|
| Battery | Hybrid | SOC < 60 && P_load > 1500 && after(1s) |
| Hybrid | Battery | SOC > 65 && after(1s) |
| Hybrid | Engine | SOC ≤ 30 && after(1s) |
| Engine | Hybrid | SOC > 35 && after(1s) |

The `after(1, sec)` guard on every transition prevents rapid chattering at the threshold boundaries.

---

## Components

| Block | Type | Purpose |
|---|---|---|
| `Battery` | Controlled Current Source (Simscape) | Models battery as electrical power source |
| `Engine` | Controlled Current Source (Simscape) | Models engine as electrical power source |
| `Resistor` | Simscape Electrical — 1 Ω | Load / network element |
| `Electrical Reference` | Simscape | Common ground for the network |
| `Current Sensor × 2` | Simscape | Measures current in the network |
| `Voltage Sensor × 2` | Simscape | Measures voltage in the network |
| `Chart` | Stateflow | EMS logic — Battery / Hybrid / Engine |
| `Repeating Sequence` | Source | Time-varying load profile |
| `Integrator` | Continuous (IC = 80) | Accumulates SOC |
| `Gain1` | 1/3600 | Converts per-second to per-hour for SOC |
| `Gain` | −1 | Sign inversion in SOC path |
| `Saturation` | 0 to 100 | Clamps SOC to valid physical range |
| `Low-Pass Filter` | τ = 5, Gain = 1 | Smooths SOC signal |
| `Capacity` | Constant = 5 Ah | Battery capacity |

---

## Simulation Parameters

| Parameter | Value |
|---|---|
| Initial SOC | 80 % |
| Battery max power (`P_batt_max`) | 1500 W |
| Smoothing factor `k` | 0.2 |
| Network resistance | 1 Ω |
| Load profile times (s) | 0, 5, 10, 15, 20 |
| Load profile values (W) | 1000, 1500, 2000, 1800, 2200 |
| SOC saturation limits | 0 – 100 % |
| LPF time constant | 5 |
| Simulation duration | 3000 s |

---

## Results

**Engine Power** starts near zero, rises during the initial phase as the EMS transitions out of the Battery state, then stabilizes into a repeating ripple pattern that mirrors the 20-second load cycle.

**Battery Power** starts at its highest level and decays steadily over time, approaching near-zero by the end of the simulation. A distinct change in slope is visible around 200–300 s, consistent with the SOC dropping below 60% and the controller entering the Hybrid state.

**SOC** decays smoothly from 80% to approximately 5% over 3000 seconds. The smooth profile confirms the Low-Pass Filter and Saturation blocks are functioning correctly. The rate of decay visibly slows in the later stages as the EMS reduces battery reliance in the Hybrid and Engine states.

The overall trend — battery power dominant early, engine power dominant late — directly validates the SOC-threshold-based mode-switching logic implemented in the Stateflow chart.

---

## File Structure

```
HEMS-EV/
├── HEMS_in_EV_Stateflow_Upgraded.slx   # Main Simulink model
├── README.md                            # This file
└── HEMS_Project_Report_GitHub.pdf       # Full technical report
```

---

## Getting Started

**Requirements:**
- MATLAB R2023a or later
- Simulink
- Simscape + Simscape Electrical toolbox
- Stateflow toolbox

**To run:**
1. Clone or download this repository
2. Open `HEMS_in_EV_Model.slx` in MATLAB/Simulink
3. Press **Run** (simulation time: 3000 s)
4. Observe outputs on `Battery_Engine_Power_Output` (Engine Power / Battery Power) and `SOC_Scope` (SOC trajectory)

---

## Documentation

Full technical report with system architecture, block parameters, Stateflow logic, and simulation results:

📘 [HEMS\_Project\_Report\_GitHub.pdf](HEMS_Project_Report.pdf)

---

## Future Scope

- Replace fixed SOC thresholds with an optimization-based layer (MPC, ECMS, or Dynamic Programming) to approach minimum-fuel operation
- Validate against standard drive cycles (NEDC, WLTP) instead of the hand-defined repeating sequence
- Add a detailed battery model with internal resistance, OCV–SOC curve, and thermal effects
- Tune smoothing factor `k` and SOC thresholds via simulation-based sensitivity analysis
- Explore fuzzy logic or reinforcement learning–based EMS variants

---

## References

1. Wirasingha & Emadi — *Classification and Review of Control Strategies for PHEVs*, IEEE Transactions on Vehicular Technology, 2011
2. Xue et al. — *A Comprehensive Review on HEV Energy Management Strategy*, Energies, 2020
3. Sabri, Danapalasingam & Rahmat — *A review on HEV architecture and EMS*, Renewable and Sustainable Energy Reviews, 2016
4. Enang & Bannister — *Modelling and control of HEVs (A comprehensive review)*, RSER, 2017
5. Bakht et al. — *Stateflow-Based EMS for Hybrid Energy Systems*, Applied Sciences, 2021

---

## License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.
