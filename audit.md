# MCL Codebase Audit Report

## 1. Executive Summary
* **System Status:** [Functionally Compromised / Arcade-Style Implementation]
* **MCL Fidelity Score:** 35/100
* **Primary Bottleneck:** **Degenerate Particle State** - The filter does not estimate heading ($\theta$). It relies 100% on the IMU, effectively reducing the Particle Filter to a simple 2D position jitter smoother. This is not "Probabilistic Robotics"; this is "sensor fusion with extra steps."

## 2. Critical Mathematical Flaws (Showstoppers)

### A. The "Stateless Theta" Fallacy
**Violation:** Probabilistic Robotics, Thrun et al., Chapter 8.3 (Grid Localization) & 8.4 (MCL).
**Location:** `include/localization/particleFilter.h` Lines 17-18, 64, 99.
**Analysis:**
You defined particles as `std::array<float, 2>` (only X and Y). You then force-feed the IMU heading into every single particle via `angleFunction().getValue()`.
```cpp
// particleFilter.h line 64
particles[i] = Eigen::Vector3f(this->particles[i][0], this->particles[i][1], angle.getValue());
```
**Consequence:**
*   You are **NOT** filtering the heading. If the IMU drifts by 5 degrees, your **entire** particle cloud rotates by 5 degrees instantly.
*   The filter cannot "correct" the heading using wall data. A valid MCL would punish particles with wrong headings because their laser scans wouldn't match the walls. Yours cannot do this because all particles have the exact same "god-mode" heading.
*   **Verdict:** This defeats 50% of the purpose of MCL.

### B. Naive Motion Model
**Violation:** Thrun Chapter 5.4, *Odometry Motion Model*.
**Location:** `src/localization/mcl_task.cpp` Lines 401-412.
**Analysis:**
You are applying independent Gaussian noise to `dx` and `dy` based on distance traveled.
```cpp
float noise_std = movement * MCLConfig::LINEAR_NOISE;
dx += noise(rng);
dy += noise(rng);
```
**Why this fails:**
Real robots don't just "teleport" X/Y error. They suffer from:
1.  **Rotational drift** (one side drags).
2.  **Arcing** ( `rot1` error).
3.  **Linear slip** ( `trans` error).
You must implement the standard `sample_motion_model_odometry` (Thrun Table 5.6) which decomposes movement into:
1.  Variable rotation 1 (turn to target)
2.  Variable translation (move to target)
3.  Variable rotation 2 (turn to final heading)
Your current model generates a spherical error blob, not the correct "banana-shaped" posterior.

### C. Sensor Model "Max Range" Suicide
**Violation:** Thrun Chapter 6.3, *Beam Models of Range Finders*.
**Location:** `include/localization/distance.h` Line 45 & `particleFilter.h` Line 78.
**Analysis:**
```cpp
// distance.h
exit = measuredMM >= 9999;
if (exit) { return std::nullopt; }

// particleFilter.h
if (auto weight = sensor->p(particle); weight.has_value() ...)
```
If a sensor sees "infinity" (nothing), you treat it as `nullopt` and **ignore it**.
**Correction:** Seeing "nothing" is extremely valuable information! It tells the robot "I am NOT near a wall." By ignoring max-range readings, you allow particles to drift into the middle of the field (where they should see nothing) while actually believing they are next to a wall. You fail to punish false positives.

## 3. Embedded Systems & Performance Risks

### A. Race Conditions & Data Tearing
**Location:** `MCLTask::getPose()` vs `MCLTask::updateLoop()`
**Analysis:**
`MCLPose` is a struct of 3 floats (12 bytes).
*   Writer: `updateLoop` writes `m_currentPose.x/y/theta` sequentially.
*   Reader: `getPose` returns `m_currentPose` by value.
On a 32-bit Cortex-A9 (Vex V5), this copy is **NOT atomic**.
**Risk:** If `getPose()` triggers whilst `updateLoop` is writing, you could get `New X` with `Old Y`. This "tearing" creates phantom glitches in the autonomous routine that are impossible to debug.
**Fix:** Use a `pros::Mutex` or `std::atomic<MCLPose>` (if structure padding allows lock-free, unlikely).

### B. Trig-Heavy Ray Casting
**Location:** `include/localization/distance.h` Lines 55-69.
**Analysis:**
Inside `sensor->p()`, you perform:
*   4 calls to `cos()`
*   4 calls to `abs()`
*   Multiple float divisions
Rate: 250 particles * 4 walls * 50Hz = **50,000+ trig/div operations per second**.
While the V5 (667MHz) can handle this, it causes jitter in the FreeRTOS scheduler.
**Optimization:** You are ray-casting against an axis-aligned bounding box (the field). You do not need `cos()`. Simple similar triangles or `vector_t` scaling works faster.

## 4. Hardware Abstraction & Code Structure

### A. The "Cheap" PDF
**Location:** `src/utils/utils.cpp`
**Analysis:** `cheap_norm_pdf` uses a heavy-tailed approximation ($1/x^4$).
*   *Actually...* I'll allow this. The heavy tail adds robustness against the "Kidnapped Robot" problem (Thrun calls this adding a uniform distribution $z_{rand}$). It prevents particle starvation. Good job accidentally implementing a robustness feature.

### B. Dynamic Allocation Abuse
**Location:** `mcl_task.cpp`
**Analysis:** `createPredictionFunction` returns a `std::function` containing a lambda. This often triggers small heap allocations depending on the size of the capture group (`dx, dy`). Doing this 50 times a second contributes to heap fragmentation over a 2-minute match.

## 5. The "Insane Detail" Nitpick List

*   **Magic Numbers:** `fieldDist{-1.78308, 1.78308}`. Defining field dimensions in meters in the header with 5 decimal places? Put this in `config.h`.
*   **Unit Confusion:** `predicted = 50.0f` hardcoded max range in meters? But `measured` is converted to `QLength`? Inconsistent abstraction level.
*   **Static Assert Lie:** `static_assert(L <= 500)` yet you use 250. Why assert 500? Just assert `type_limits` or RAM constraints.
*   **Incorrect Ray Math:** `std::min((WALL_0_X - x.x()) / cos(theta), predicted)`. If `cos(theta)` is near 0, this explodes to infinity. `std::min` catches it, but the division by zero risk exists if `theta` aligns perfectly with grid lines (which it does, often).

## 6. Recommendations for Refactor

1.  **Promote Theta to Particle State:** Change state to `[x, y, theta]`. Filter heading. This is the only way to fix gyroscope drift using wall geometry.
2.  **Implement Thrun's Odometry Model:** Replace `dx/dy` noise with `rot1/trans/rot2`.
3.  **Atomic State Access:** Protect `m_currentPose` with a mutex.
4.  **Lookup Tables:** Pre-compute `cos/sin` or use `lookup_table` for the ray casting logic.
5.  **Fix Measurement Model:** Handle `max_range` readings by giving a small constant probability (e.g., $1/\text{field\_diag}$) instead of discarding them.
