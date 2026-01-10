# Installation
simply follow the mujoco-unitree installation documentation from (https://github.com/unitreerobotics/unitree_mujoco)

### Side note :: I installed the dependencies on anaconda env

# Launch the simulator 
- launch env
- `cd unitree_mujoco/simulate_python`
- launch ```python3 unitree_mujoco.py```

# Separate terminal, launch the controller
1. launch env
2. `cd unitree_mujoco/example/python`
3. launch ```python3 diagonal_pair_step_lowcmd.py```
4. files can be found on python folder (https://github.com/Fraulainee/mujoco_examples)

### Visit this link to understand four legged movement
https://www.youtube.com/watch?v=WrR3fVQ3W3s

# `diagonal_pair_step_lowcmd.py` — MuJoCo Unitree LowCmd Gait (4-Beat Walk + Rear Stance Push)

## Overview

This script drives a Unitree quadruped model in **MuJoCo** using the **Unitree SDK2py LowCmd** interface. It publishes joint targets at a fixed control rate and generates a simple open-loop gait:

- **Phase 1:** Stand up into a stable pose (2 seconds)
- **Phase 2:** A repeating **4-beat walk** sequence: **RR → FR → RL → FL**
  - Each leg alternates between **swing** (foot in air) and **stance** (foot on ground)
  - During **stance**, the thigh is swept in a way that creates a **forward “push”** (rear legs push more than front legs)

This is a **joint-space, PD-controlled, open-loop** gait generator—meant for repeatable motion in simulation (not a full balance controller).

---

## Key Ideas (Why it’s built this way)

### 1) Open-loop joint-space gait
Instead of computing foot trajectories + IK, we directly command **thigh/calf angle offsets** around a stable **stand pose**. This is simpler and robust for initial gait experiments.

### 2) Separate swing vs stance gains
- **Swing** uses softer gains to reduce jitter while the leg is moving through the air.
- **Stance** uses stronger gains to hold the leg firmly against ground contact and make the “push” effective.

### 3) Stance “push” for forward motion
A pure swing-only motion often causes stepping in place or slipping. So stance includes a **backward sweep** of the thigh to produce traction-based forward motion.

### 4) Touchdown hold to reduce slipping
Immediately after swing ends, the foot is most likely to slip at contact. A short **hold** reduces the sudden commanded change right at touchdown.

---

## Requirements

- Python 3
- `unitree_sdk2py` installed and available
- A running MuJoCo simulation compatible with Unitree LowCmd DDS topics

---

## Topics / Communication

The script publishes:

- **Topic:** `rt/lowcmd`
- **Message type:** `LowCmd_`

It uses:
- `ChannelFactoryInitialize(1, "lo")` → initializes DDS on loopback interface
- `ChannelPublisher("rt/lowcmd", LowCmd_)` → publisher object

---

## Constants and Configuration

### Control rate
- `DT = 0.005` → 200 Hz command publish rate

### Gains (PD)
Two gain sets:

| Phase  | KP | KD | Purpose |
|-------|----|----|---------|
| Swing | `KP_SWING=55` | `KD_SWING=2` | softer tracking while moving leg through air |
| Stance | `KP_STANCE=85` | `KD_STANCE=3` | firmer contact + better push |

> If the robot slips a lot, reduce `KP_STANCE` (and/or reduce stance push).

### Stand pose
Per leg: `[hip, thigh, calf] = [0.0, 0.8, -1.6]`

The full 12-joint pose:
- `STAND_Q = STAND_LEG * 4`

This is the “neutral reference” pose. Gait offsets are added on top.

### Actuator mapping
- `NUM_ACT = 12` → 12 actuated joints
- `NUM_MOTOR_SLOTS = 20` → LowCmd message has 20 slots; only first 12 are used

Index map:

- FR: 0..2
- FL: 3..5
- RR: 6..8
- RL: 9..11

`LEG_JOINTS` groups names per leg.

---

## Gait Timing

This gait is driven by a normalized global phase:

```python
global_phase = (t % STEP_PERIOD) / STEP_PERIOD
```

## Timing parameters

- `STEP_PERIOD = 1.20` seconds per full gait cycle
- `SWING_PORTION = 0.32` fraction of each leg cycle spent in swing
- `OVERLAP = 0.05` extra swing window margin to smooth transitions
- Swing window length:
   - `swing_window = SWING_PORTION + OVERLAP`

## Leg phase offsets (4-beat order)
A leg’s local phase is:
```
local = (global_phase01 - LEG_PHASE[leg]) % 1.0
```

Offsets:
- RR: 0.00
- FR: 0.25
- RL: 0.50
- FL: 0.75

This creates the repeating stepping order:
``` RR → FR → RL → FL ```


## Motion Parameters
### Swing (thigh move + lift)

Front legs:
- `FRONT_SWING_AMP = -0.30`
- `FRONT_LIFT_AMP = 0.52`

Rear legs:
- `REAR_SWING_AMP = -0.26`
- `REAR_LIFT_AMP = 0.46`

Additional thigh lift coupling:
- `THIGH_LIFT_AMP = 0.10`

### Stance push (the “engine”)

Rear legs push more:
- `REAR_STANCE_PUSH = 0.22`
- `FRONT_STANCE_PUSH = 0.14`

Touchdown hold
- `TOUCHDOWN_HOLD = 0.06 (fraction of leg cycle)`
   This adds a short “do nothing” window after swing ends before stance sweep begins.

## Function-by-Function Documentation
```
zeros_for_field(msg_cls, field_name, default_len=4)
```

Attempts to create a zero-filled list matching the true fixed-array length of a dataclass field in Unitree IDL messages.

Why this exists:
Unitree DDS messages often contain fixed-length arrays (`head`, `sn`, `version`, reserves, etc.). Different SDK builds may enforce those sizes strictly. This helper reduces version mismatch issues.

Returns:
- a list of zeros of correct length (best effort), else `default_len`

---

`make_motor_cmd() -> MotorCmd_`

Creates a `MotorCmd_` with safe default values and correctly-sized `reserve`.

Used to populate `LowCmd_.motor_cmd` which expects a list of motor command slots.

---

`make_bms_cmd() -> BmsCmd_`

Constructs `BmsCmd_` robustly across SDK versions.

Problem it solves:
Some versions of `BmsCmd_` require specific constructor fields. This function introspects fields and fills:
- fixed arrays with zeros
- sequences with empty lists
- scalar fields with zero
