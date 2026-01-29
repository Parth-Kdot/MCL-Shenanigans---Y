# Simple Field Test Protocol

**Goal:** Verify the robot knows where it is.
**Tool:** The "Matrix" Data Stream (Lines of text scrolling in the terminal).

---

## 1. The Screen Setup

Connect the robot to the computer. Open the Terminal.
You will see lines appearing like this:
```text
DATA, 10500, 12.0, 10.0, 12.1, 9.8, 0.050, SAFE
```

### How to Read It
*   **Column 1 (TIME):** Just a clock. Ignore it.
*   **Column 2 & 3 (ODOM):** Where the **Wheels** think they are. (The Reference).
*   **Column 4 & 5 (MCL):** Where the **Brain** thinks it is. (The Test Subject).
*   **Column 6 (SIGMA):** The "Confusion Score".
    *   **Low (0.00 - 0.10):** Confident. "I know exactly where I am."
    *   **High (> 0.50):** Confused. "I am lost."

---

## 2. Test 1: The Box (Wheels Only)
**Setup:** Put the robot on a box (wheels spinning freely in air).
**Action:** Drive forward with the joystick.

| What to Watch | Expected Result | Why? |
| :--- | :--- | :--- |
| **Col 2 (ODOM)** | **Changes** (e.g., 0 -> 24) | Wheels are turning. |
| **Col 4 (MCL)** | **Changes** (e.g., 0 -> 24) | The Brain listens to the wheels first. |
| **Col 6 (SIGMA)** | **Increases** (0.01 -> 0.05) | The Brain gets slightly unsure because it can't see walls moving. |

** PASS Condition:** MCL follows Odom closely.

---

## 3. Test 2: The Push (Sensors Only)
**Setup:** Put the robot on the floor.
**Action:** **Push** the robot sideways (or forward) *without* spinning the wheels (the wheels drift/slide).

| What to Watch | Expected Result | Why? |
| :--- | :--- | :--- |
| **Col 2 (ODOM)** | **Stays Same** (0.0) | Encoders didn't move! |
| **Col 4 (MCL)** | **CHANGES** (0 -> 5.0) | The Brain sees the wall getting closer/further! |

** PASS Condition:** MCL moves while Odom stays still. This proves the sensors are working.

---

## 4. Test 3: The Spin (Calibration)
**Setup:** Robot on floor, center of tile.
**Action:** Spin in place 360 degrees.

| What to Watch | Expected Result | Why? |
| :--- | :--- | :--- |
| **Col 6 (SIGMA)** | **Goes Up** (0.01 -> 0.15) | Spinning causes wheel slip. The Brain *should* get slightly confused. |
| **MCL Position** | **Stays roughly (0,0)** | You are only spinning, not moving. |

** PASS Condition:** Sigma increases but doesn't explode (> 1.0).

---

## 5. Troubleshooting (Cheat Sheet)

*   **Problem:** Robot teleports instantly to strict (0,0) or (50,50).
    *   **Fix:** Check `config.h`. Your Sensor Offsets might be huge/wrong.

*   **Problem:** Robot thinks it's at (0,0) but Odom says (100, 100).
    *   **Fix:** Your sensors are unplugged or returning 0. The filter kills particles that are far away, so it freezes.

*   **Problem:** "SIGMA" is always 999.0.
    *   **Fix:** You haven't initialized. Run the code again.
