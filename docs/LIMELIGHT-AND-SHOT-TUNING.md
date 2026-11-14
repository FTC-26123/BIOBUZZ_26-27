# Limelight, shot speed and shooting range: how to tune

Do these in order, on the practice field, after the flywheel is tuned (ROBOT-SETUP.md steps 3 to 5).
Every number goes into **Robot Settings** on the robot, so no rebuild is needed. Write your results in the worksheet at the end.

What you end up with:
1. A Limelight that reads distance correctly (within ±1 in).
2. The **exact flywheel speed** for every distance you shoot from (the shot table).
3. The **exact closest and farthest distance** you can score from (`Min shot dist in` / `Max shot dist in`).
4. Auton shot speeds that are known to work from the auton shot marks.

> **Which distance?** Always use the **`Dist`** number on the `Shot Table Builder` screen (first number on the
> line `Dist NN in (tags NN)`). The shot table, the range check and the LEDs all use that number. Don't use a tape
> measure for these steps. The tape is only for calibrating the camera in step 1.

---

## 1. Limelight setup and calibration

Do ROBOT-SETUP.md **9.1 to 9.12** (mount, pipeline, exposure, calibration, UP/DOWN heights). In short:

- [ ] Pipeline: AprilTag, family 36h11, tag size 3.25 in, Full 3D. Robot Settings > Camera > `Pipeline` = that slot.
- [ ] Exposure: from your **farthest** spot all four tags stay outlined, and they don't blur when the robot turns.
- [ ] Height (12 3/8 in) and left/right (centred) are already in the code. **Pitch and forward are calibrated, not
      measured:** run `Limelight Calibration (pitch + forward)`, press A with the robot centre on a 30 in tape mark and
      Y on a 78 in mark (square to the cell, all 4 tags in view), check the two tag heights agree, press START to save.
- [ ] Check: `Shot Table Builder` reads `tags 30` and `tags 78` (± 1) on the marks.
- [ ] Recheck after any bump to the camera mount. A bent mount shows up as a distance error at 78 in.

## 2. Limelight check in every auton (read it before every match)

Every auton runs a **LIMELIGHT CHECK** during INIT, from the start tile. If it fails, the LEDs flash gold.
The robot still runs if the check fails, but the screen tells you what is wrong:

| Screen says | Meaning / fix |
| --- | --- |
| `OK` | Camera streaming, right pipeline, sees our cell in the expected state, distance right |
| `NOT FOUND` | Not in the robot configuration or unplugged. Check the USB cable and the `limelight` name |
| `NO FRAMES` | Found but not streaming. Wait 5 s after INIT, then re-plug the cable |
| `WRONG PIPELINE n` | Robot Settings > Camera > `Pipeline` doesn't match the Limelight slot |
| `OLD FRAMES` | Frames arriving late: cable, or the Control Hub is overloaded |
| `NO RED/BLUE TAGS IN VIEW` | Camera aimed wrong, exposure too low, or the robot isn't on its tile |
| `SEES THE OTHER CELL` | Robot on the wrong tile, or the wrong side's program was picked |
| `OUR CELL LOOKS DOWN but this is a UP-side program` (or the reverse) | **You probably picked the wrong program.** Pick the other side's program |
| `DISTANCE OFF BY ±n in` | Robot isn't centred on its tile, or the camera needs recalibrating (step 1) |

The distance check runs only when our cell is UP. From the Up start tile the expected distance is about **49 in**
(the screen shows `expected`). The allowed error is Robot Settings > Auto camera > `INIT check tol in` (default 4).
On the Down side, the raised cell across the frame can block the view of ours. The check then says so and doesn't fail.

## 3. Find the exact speed for each distance (shot table)

Use `Shot Table Builder`, with our cell UP.

- [ ] **3.1** First set the shot model so the starting speeds are close (ROBOT-SETUP.md 9.18 to 9.22: launch angle,
      exit height, scale at a near and a far spot).
- [ ] **3.2** Pick 5 spots across the range you shoot from, about 12 in apart in `Dist`. Include the two auton shot
      marks (step 5). At each spot:
  1. Hold G1 LT until `AIMED`. Keep holding it while you shoot.
  2. **Find the speed window.** Start 100 below `model`. Fire 5 balls. Raise the speed one step (G2 Dpad Right,
     +25 ticks/s) and fire 5 again. Keep going until balls go over or bounce out.
     - **Low edge** = the lowest speed where at least 4 of 5 score.
     - **High edge** = the highest speed where at least 4 of 5 score.
  3. The **exact speed** is the middle of the window, rounded to 25. Set it with Dpad Right/Left.
  4. Confirm it: 10 balls, at least 8 must score. If not, the window is too narrow at this distance. Note it, because it's near
     the edge of your range (step 4).
  5. Press **G2 A** to record the row (distance and speed). To redo a row, stand there and press **B**.
- [ ] **3.3** Check in between: stand halfway between two recorded spots. With AIM held, `final` should be between the two rows' speeds,
      and 8 of 10 should score with no manual change. If not, record a row there too.
- [ ] **3.4** Battery check: repeat one spot near 12.2 V. If it misses, re-tune the flywheel (ROBOT-SETUP.md step 3)
      before changing rows.

Write each spot's low edge, high edge and recorded speed in the worksheet. A narrow window (less than 50 ticks/s) means
that distance is risky in a match.

## 4. Find the exact closest and farthest distance you can score from

The LEDs and the "can we score from here" check use Robot Settings > Aim > `Min shot dist in` and `Max shot dist in`.

- [ ] **4.1 Closest:** start at your closest recorded spot. Move **6 in closer** (by `Dist`) and do step 3.2.2 to
      3.2.4 there. Repeat until no speed gets 8 of 10 in. The ball hits the lip, or the robot is under the cell.
- [ ] **4.2** Go back halfway (3 in), then 2 in, to find the closest spot that still gets 8 of 10. That `Dist` is
      your **exact minimum**. Record a row there (G2 A).
- [ ] **4.3 Farthest:** same thing going **back** 6 in at a time from your farthest spot. Stop when no speed gets 8 of 10,
      or the flywheel can't reach the speed (`Speed now` never reaches the target). Halve back
      to the farthest spot that still gets 8 of 10. That is your **exact maximum**. Record a row there.
- [ ] **4.4** Set `Min shot dist in` = exact minimum **+ 2** and `Max shot dist in` = exact maximum **- 2**. The
      2 in margin covers camera noise.
- [ ] **4.5** Check it: walk slowly from too close to too far with AIM held. The LEDs only show "in range" between the
      two numbers, and every in-range spot scores 8 of 10 with no manual change.

The shot table also refuses distances outside its first and last rows when the model can't make the shot. That's
why rows at the exact minimum and maximum matter.

## 5. Auton shot speeds and shot marks

- [ ] **5.1** Put the robot on each auton shot mark (run the auton once and stop it at the shot, or place it from
      the field numbers in ROBOT-SETUP.md 10.1). Read `Dist`. It must be between your min and max from step 4.
      If it isn't, the shot mark needs moving (code change in `CycleRoute` / `DelayedBasketTwoRoute`).
- [ ] **5.2** Do step 3.2 there and set Robot Settings > Auto shooting > `UP shot speed` / `DOWN shot speed` to
      that exact speed. The auton uses these when the camera can't see the cell. With the camera on and at least one row recorded,
      it uses the table instead.
- [ ] **5.3** Run each auton's shoot plan 5 times: `Balls counted` = 4 every time, and the balls score.

---

## Worksheet

Limelight: pipeline slot ____  ·  `tags` at 30 in tape ____  ·  at 78 in tape ____  ·  UP height ____ in  ·  DOWN height ____ in

| Spot | `Dist` (in) | Low edge | High edge | Recorded speed | 8/10? |
| --- | --- | --- | --- | --- | --- |
| Exact minimum | | | | | |
| 1 | | | | | |
| 2 | | | | | |
| Up auton shot mark | | | | | |
| Down auton shot mark | | | | | |
| 3 | | | | | |
| Exact maximum | | | | | |

`Min shot dist in` = ____ (min + 2)  ·  `Max shot dist in` = ____ (max - 2)  ·  `UP shot speed` = ____  ·  `DOWN shot speed` = ____
