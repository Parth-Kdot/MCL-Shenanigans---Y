# Vex V5 MCL Field Test Manual ("The Pilot's Handbook")

**Version:** 2.0 (Thrun Motion Model Edition)  
**Target Platform:** Vex V5 (Cortex-A9)  
**System:** Monte Carlo Localization (Particle Filter)

---

## 1. The Core Update: Thrun's Motion Model

We have upgraded the robot's "imagination". Previously, the robot thought movement error was just a fuzzy circle around its position. Now, it understands **Kinematic Slip**.

**What this means:**
The robot decomposes every movement into three steps:
1.  **Rotation 1:** Turning toward the target.
2.  **Translation:** Driving to the target.
3.  **Rotation 2:** Turning to the final heading.

Error is applied to *each* step individually.

---

## 2. Field Testing Protocols

### Test A: The "Kidnapped Robot" Recovery
1.  **Start** the robot in the center of the field.
2.  **Cover** the sensors (or stand in front of them) so it sees nothing.
3.  **Move** the robot manually to a corner (do not drive it, pick it up).
4.  **Uncover** the sensors.
5.  **Observation:** The particles should initially be scattered. Within **2-3 seconds** of seeing walls, the particles should "snap" to the corner location.
6.  **Success Condition:** Variance drops below `0.1`.

### Test B: The "Spin Test" (Calibrating Rotational Noise)
**CRITICAL CHANGE:**
In the old system, spinning in place kept the X/Y position perfect.
In the **new system**, spinning in place **WILL** cause the particles to spread out slightly in X/Y.

*   **Why?** Real tank-drive robots "wobble" when they turn. The tread grip varies.
*   **Good Behavior:** After 10 spins, the cloud should be a small fuzzy ball (approx 4-inch radius) around the center.
*   **Bad Behavior:**
    *   *Cloud explodes to the whole field:* `ANGULAR_NOISE` is too high.
    *   *Cloud stays a perfect single pixel:* `ANGULAR_NOISE` is zero (bad! Robot will be overconfident).

---

## 3. Tuning Matrix

Use this table to tune `MCLConfig` in `include/localization/mcl_task.h`.

| Symptom | Diagnosis | Tuning Action |
| :--- | :--- | :--- |
| **Robot drives straight**, but particles spread out sideways too much. | Overestimating drift. | **Decrease** `LINEAR_NOISE`. |
| **Robot drives straight**, but particles lag behind or jump ahead. | Underestimating slip. | **Increase** `LINEAR_NOISE`. |
| **Robot turns**, and the particle cloud "explodes" in size. | Imagining too much wobble. | **Decrease** `ANGULAR_NOISE`. |
| **Robot turns**, but particles don't spread at all. | Overconfident (Dangerous). | **Increase** `ANGULAR_NOISE`. |
| **Robot hits a wall** but thinks it is 6 inches away. | Sensor Model too weak. | **Increase** `LOCO_CONFIG::DISTANCE_WEIGHT` in `config.h`. |
| **Robot sees a ghost** (opponent) and jumps position wildly. | Sensor Model too aggressive. | **Decrease** `LOCO_CONFIG::DISTANCE_WEIGHT`. |

---

## 4. Diagnostics Dashboard

When debugging via terminal, look for the "Converged" flag:

```text
[MCL] Update #500: (45.20, -12.10) var=0.0402 CONVERGED
```

*   **x, y:** Position in inches.
*   **var:** Variance in meters.
    *   `< 0.1`: Excellent lock.
    *   `0.1 - 0.5`: Good tracking.
    *   `> 0.5`: Lost / Searching.
