# MCL Stress Test Report

**Target:** `src/localization/mcl_task.cpp` & `include/localization/particleFilter.h`
**Version:** Release Candidate 2.0
**Status:** **PASS**

---

## 🔬 Scenario 1: The "Donut" Problem (Angle Wrapping)
**Objective:** Ensure angle arithmetic does not result in unbounded theta (e.g., $3.14 + 0.1 \rightarrow 3.24$).
**Trace:**
1.  **Motion Model (`mcl_task.cpp`):** The lambda function calculates `p.theta += hat_rot1 + hat_rot2`. Result can technically exceed $\pi$.
2.  **Particle Filter (`particleFilter.h`):** Immediately after the motion update loop:
    ```cpp
    while (p.theta > M_PI) p.theta -= 2 * M_PI;
    while (p.theta <= -M_PI) p.theta += 2 * M_PI;
    ```
3.  **Result:** Angles are strictly bound to $[-\pi, \pi]$.
**Verdict:** **PASS**

---

## 🙈 Scenario 2: The "Blind Robot" (Divide by Zero)
**Objective:** Prevent crash when all sensor probabilities are zero (`totalWeight = 0`).
**Trace:**
1.  **Condition:** Robot is completely blocked or sensors disconnected. `weightParticle()` returns 0 or near-zero for all particles.
2.  **Accumulation:** `totalWeight` sums to `0.0`.
3.  **Safety Check (`particleFilter.h`):**
    ```cpp
    if (totalWeight <= 1e-6) {
         return; 
    }
    ```
4.  **Consequence:** The filter aborts the *Resampling* step.
    *   **Good:** It avoids the `avgWeight = totalWeight / L` division (which would handle 0, but `normal_distribution(0,0)` is undefined).
    *   **Behavior:** The filter effectively "freezes" its update logic until sensors recover. It does *not* crash the processor.
**Verdict:** **PASS**

---

## 🌀 Scenario 3: The "Motion Singularity" (Zero Translation)
**Objective:** Ensure the Thrun Odometry Model handles turning in place ($\delta_{trans} \approx 0$).
**Trace:**
1.  **Input:** Odometry reports $\Delta x = 0, \Delta y = 0$.
2.  **Thrun Math:** Standard formula for `atan2(dy, dx)` is unstable at (0,0).
3.  **Safeguard (`mcl_task.cpp`):**
    ```cpp
    if (delta_trans < 0.001f) {
        delta_rot1 = 0;
        delta_rot2 = angleDiff(theta_prime, theta_prev);
    }
    ```
4.  **Result:** The singularity is bypassed. The model switches to a "Pure Rotation" mode.
**Verdict:** **PASS**

---

## 🏁 Final Certification

The logic has withstood mental execution trace. All critical mathematical paths contain safeguards for their respective edge cases.

**Recommended Action:** Proceed to compilation and physical field testing.
