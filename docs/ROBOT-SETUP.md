# BIOBuzz robot: controls, setup and checklists

Last updated 2026-11-11. This one file replaces all the earlier notes.

**Status of the code:** everything described here was written WITHOUT the robot. It builds against the real
FTC SDK 12.0, but it has never been run on real hardware. Part C is the list of things to do, in order, to get
from here to a working robot.

**No more editing code to tune.** Every number in this guide is changed on the robot from the Driver Station
and saved on the Control Hub (`/sdcard/FIRST/biobuzz-settings.properties`). The next OpMode you run uses it.
You only build and deploy once. The numbers written in the code are just the starting defaults. See A6.

All file names below are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

---

# Part A. Controls

## A1. Autonomous: one program per alliance, side and plan

Pick it on the Driver Station; there is no menu. Names: `<Blue|Red> <Up|Down> - <Shoot + Flower + Park | Shoot + Park | Park Only>`
(group `26123 Blue` / `26123 Red`). **Up** = our cell is raised at the start; **Down** = it is down.
Every program on one side starts from the **same spot**: the robot centred on the wall tile in front of that cell,
facing it. Each one preselects the matching `Blue TeleOp` / `Red TeleOp`.

Read the INIT screen before Play: alliance, side, plan, START, `LIMELIGHT CHECK` (must say OK; see
LIMELIGHT-AND-SHOT-TUNING.md step 2), flower arm.

## A2. TeleOp

Pick `Blue TeleOp` or `Red TeleOp` (plus `One Controller` / `Speed Adjust` versions). The alliance is set by the
program, there is nothing to pick in INIT. After an auton the right one is already selected.

## A3. Two players: `TeleOp_BaseCode` and `TeleOp_BaseCodeSpeedAdjust` (identical)

Gamepad 1, driver

| Control | Action |
| --- | --- |
| Left stick | Drive forward/back and strafe |
| Right stick X | Turn |
| Dpad Right | Drive speed 25 / 50 / 75 / 100 % |
| Left trigger (hold) | AIM: turn to our UP cell's aim point and set speed from distance. With the left stick still, the robot's spot stays locked (it cannot be pushed off it) while it turns. |
| Right stick button (hold) | Bypass position hold |

Gamepad 2, operator

| Control | Action |
| --- | --- |
| Right trigger (hold) | FIRE: balls feed one at a time, each only when the flywheel is at speed |
| Right trigger released | Stop feeding; flywheel keeps spinning |
| Y | Spin the flywheel up without feeding |
| Right bumper | Flywheel off (coasts down) |
| Dpad Up / Down | Next / previous speed preset (first press leaves AUTO speed) |
| Dpad Right / Left | Trim speed +25 / -25 |
| X | Clear trim |
| Left trigger | Intake in |
| Back | Eject (everything in reverse) |
| Left bumper | Intake stop |
| A | Flower arm: toggles DOWN / UP |

## A4. One player (everything on gamepad 1)

| Control | `TeleOp_BaseCodeOneController` | `TeleOp_BaseCodeOneControllerSpeedAdjust` |
| --- | --- | --- |
| Left stick | Drive and strafe | same |
| Right stick X | Turn | same |
| Left stick button (hold) | AIM | same |
| Right stick button (hold) | Bypass position hold | same |
| Dpad Right | Drive speed | same |
| Right trigger (hold) | FIRE | same |
| Right trigger released | Stop feeding; flywheel keeps spinning | same |
| Y | Spin up without feeding | same |
| Right bumper | Flywheel off | same |
| X | Clear trim | same |
| Left trigger | Intake in | same |
| Back | Eject | same |
| Left bumper | Intake stop | same |
| Dpad Up / Down | Next / previous preset | Trim +25 / -25 |
| Dpad Left | Flower arm toggle DOWN / UP | Next preset |
| B / A | Trim +25 / -25 | B: nothing, A: flower arm toggle DOWN / UP |

## A5. Changing a button

Each is one small method in the TeleOp file, with a comment above it listing alternatives.

| What | Method |
| --- | --- |
| AIM | `aimButton()` |
| Fire, spin up, flywheel off | `handleManualShootingControl` |
| Presets and trim | `handleShooterSpeedControl` (the step size is Robot Settings > Shooter speeds > `Trim step`) |
| Intake | `handleManualIntakeControl` |
| Flower arm | `handleFlowerArm()` |
| Autonomous menu | `init_loop()` in `Auton/BioBuzzAuto.java` |

Two things work differently from before:
- The fire trigger is **hold to fire**. It used to be tap to start, right bumper to stop.
- On the one-player TeleOps, holding the intake trigger no longer bypasses the position hold.

Button changes are the one thing that still needs a code change and a re-upload.

## A6. Setup and tuning OpModes (change settings without code)

### `Robot Settings (edit + save)`, gamepad 1. Works in INIT; nothing drives.

| Control | Action |
| --- | --- |
| LB / RB | Previous / next group (Shooter PIDF, Shooter speeds, Shooter gate, Intake, Flower arm, Camera, Aim, Position hold, Auto ...) |
| Dpad Up / Down | Pick a setting (hold to scroll) |
| Dpad Right / Left | Change it (hold to repeat). ON/OFF settings toggle. **Saved immediately.** |
| A / B | Bigger steps (x10) / smaller steps (/10) |
| X | Put this setting back to the code default |
| Back + Start, hold 2 s | Put EVERY setting back to the code defaults |

A `*` after a value means it differs from the code default. The flower arm is tested and tuned in its own OpMode,
`Flower Arm Test` (step 8).

### `Shooter Tuner (kS, kV, kP)`. Gamepad 1 drives as in A3. All gains start at 0.

Follows the video "PID Control Full Course: Build & Tuning for Robotic Flywheels" (Brogan Pratt):
power = **kS** (just gets the wheel turning) + **kV** x target (holds the speed) + **kP** x error (fixes drops).
No kI or kD on a flywheel.

| Gamepad 2 | Action |
| --- | --- |
| Dpad Left / Right | Step: kS, kV, kP. The flywheel is driven the way that step needs (kS = only kS; kV = kS + kV, no kP; kP = everything) |
| Dpad Up / Down | This step's gain + / - |
| A / B | Steps x10 / /10 |
| Y | Flywheel start / stop |
| X | Target HIGH / LOW (LOW = 55 % of the preset, like 1500 -> 800 in the video) |
| RB | Next preset (HIGH) |
| Left stick button | kV step only: kV helper (kV x target / 1 s average speed) |
| Right stick button | All gains to 0 |
| Hold RT | Fire a real ball (kP step) |
| LT / Back / LB | Intake in / eject / stop |
| **Start** | **SAVE** kS, kV, kP. Unsaved changes are dropped when the OpMode stops. |

Screen (5 lines): what to do now, the three gains and step size, target / speed / error / power,
`Last change: back in 0.24 s, overshoot +22`, save status.

### `Shot Table Builder`. Gamepad 1 drives as in A3 (hold LT = aim). Gamepad 2 has the A3 shooter and intake buttons, plus:

| Gamepad 2 | Action |
| --- | --- |
| Right stick up / down | Aim higher / lower in the CELL opening (0.05 per push, saved) |
| Right stick button | Scale the shot model so it gives the current speed at this distance (saved) |
| A | Record a row: the distance now and the flywheel target now (saved) |
| B | Delete the row nearest the distance now |
| Back + Start, hold 2 s | Delete all rows |

Screen (5 lines): which CELL and how many tags, distance and aim height, speed now / model / final,
the two tuning values, the rows.

### `Tune Everything (drive + shoot + settings)`: all of the above in one, for practice-field tuning

Gamepad 1 drives exactly as in A3 (hold LT = AIM). Gamepad 2:

| Always | Hold RT = fire, LT = intake, Back = eject, Y = spin up, **Start = switch SHOOT / TUNE mode** |
| --- | --- |
| SHOOT mode | RB = flywheel off, LB = intake stop, Dpad Up/Down = preset, Dpad Right/Left = trim, X = clear trim, A = record a row, B = delete the nearest row, right stick up/down = aim height, right stick button = scale the shot model here, left stick button = clear the hold's peak number |
| TUNE mode | The Robot Settings menu: LB/RB = group, Dpad Up/Down = pick, Dpad Right/Left = change, A/B = step size, X = code default. Shooter, intake, aim, aim point, shot model and position hold changes take effect immediately (no restart). Autonomous and flower arm settings apply next run. |

Screen: mode line, flywheel line, 2 aim lines, a `Hold:` line (status, how far off its spot, biggest push),
and in TUNE mode 5 menu rows. To turn the flywheel off while in TUNE mode, press Start (back to SHOOT), then RB.

### `Tune Position Hold`, gamepad 1

A = hold this spot, B = release, Dpad Up/Down = pick, Dpad Left/Right = change, bumpers = step size,
Y = clear numbers, Start = next test step, **X = SAVE**.

---

## A7. Match timing and LED colours

- **Auto-end:** autonomous stops itself **30 s** after Play, TeleOp **120 s** after Play (Robot Settings > Match;
  set `TeleOp length s` = 0 for long practice). The Driver Station also stops them in a real match.
- **Time warnings:** **30 s left** = slow white pulse (TeleOp), **10 s left** = fast white strobe (autonomous at 20 s and
  TeleOp at 110 s), each for 2 s.

| When | Colour | Means |
| --- | --- | --- |
| INIT | Alliance breathing | All good |
| INIT | Breathing alternating with flashing gold | Something missing or failing (camera / Limelight check / flower arm not up) |
| Auto | Alliance chase | Driving (shot mark, park, slide into the slot, or holding clear of the teammate) |
| Auto | Alliance heartbeat | Down side: waiting for our cell to come up |
| Auto | Solid alliance | Cell seen up / lining up a shot |
| Auto | Green | Firing |
| Auto | Violet | Flower step (including raising the arm) |
| Auto | Rainbow / flashing gold | Finished OK / finished after a problem |
| TeleOp | Flashing gold | Feed jammed (eject with Back) |
| TeleOp | Green | FIRE NOW: in range, aimed, flywheel at speed |
| TeleOp | Solid / heartbeat alliance | Basket in view: shootable / not shootable from here |
| TeleOp | Yellow / lime | Flywheel spinning up / at speed (no basket in view) |
| TeleOp | Violet | Flower arm commanded down |
| TeleOp | Dim alliance breathing | Idle |
| Both | Slow white pulse / fast white strobe | 30 s / 10 s left |

# Part B. Reference

## B1. The switches

Everything optional is OFF until you turn it on. With a switch off, that part does nothing at all. All of
them are in `Robot Settings`. In the simulator, the flower arm and the camera are always on.

| Switch (Robot Settings group > setting) | Turns on | Turn on at |
| --- | --- | --- |
| Flower arm > `Enabled` | The flower arm servo, in TeleOp and autonomous | Step 8 |
| Camera > `Camera enabled` | Limelight: AIM button, speed from distance, tip detection in autonomous | Step 9 |
| Auto camera > `Use camera` | Lets autonomous use the camera (already ON; needs the switch above too) | leave |
| Auto camera > `Camera fix position` | Autonomous corrects its odometry position from the basket tags at the shot mark | Step 10.12 to 10.14 |

## B2. Where the numbers live

| What | Where |
| --- | --- |
| Flywheel gains (kS, kV, kP) | `Shooter Tuner (kS, kV, kP)` (or Robot Settings > Shooter PIDF) |
| Presets and trim step | Robot Settings > Shooter speeds |
| Speed window, shot detection, feed timing | Robot Settings > Shooter gate |
| Windmill / transfer speeds, gate hold, jam recovery | Robot Settings > Intake |
| Flower arm direction, powers, max travel | Robot Settings > Flower arm, set by the `Flower Arm Test` OpMode (names `flowerArm`, `touchUp`, `touchDown` in the robot configuration) |
| Camera pipeline, tag ids, camera mount | Robot Settings > Camera |
| Aim gains | Robot Settings > Aim |
| Aim point height, shot model scale, table rows | `Shot Table Builder` (or Robot Settings > Aim point / Shot model) |
| Launch angle, exit height | Robot Settings > Shot model (measure on the robot) |
| Position hold | `Tune Position Hold` (or Robot Settings > Position hold) |
| Autonomous settings and assumptions | Robot Settings > Auto assumptions / Auto shooting / Auto camera / Auto DOWN side / Auto flower / Auto timing |
| Start marks, shot marks, parking slots | **Still code:** `Auton/cycles/CycleRoute.java`, `DelayedBasketTwoRoute.java` |

The saved file is `/sdcard/FIRST/biobuzz-settings.properties` on the Control Hub. Deleting it (or holding
Back + Start in Robot Settings) puts everything back to the code defaults. A new setting is added in
`subsystems/RobotSettings.java`, one line each.

## B3. What the code does now, in short

- **One ball at a time.** A ball is fed only while the flywheel is within target -40 to +80 ticks/sec. The
  moment a ball goes through and the speed drops, the gate closes: windmill stopped with BRAKE, transfer
  stopped. It reopens as soon as the speed is back. The only fixed delay is 0.05 s.
- **The old reverse logic.** While intaking, the windmill still turns slowly backwards to keep balls out of
  the shooter (unchanged). The old code also reversed it between every shot, which pushed the next ball away;
  that is what the brake-hold replaces. If a braked windmill does not hold a ball, step 4.2 says what to change.
- **Speed.** Four presets (1500, 1600, 1700, 1900) each with its own remembered trim. With the camera on and
  a shot table filled in, AIM switches the speed to AUTO (from distance); a preset button switches back.
- **Jam recovery.** Windmill bogged for 0.3 s: windmill and transfer back up for 0.1 s and retry, three
  times, then the screen shows FEED JAMMED.
- **One autonomous program** with a menu. The ten older programs are hidden, not deleted: each has
  `@Disabled` above its class in `Auton/`. Delete that line to bring one back.

## B5. Desktop simulator

The simulator in the folder next to this project (`BioBuzSim`) runs this code on a Mac or Windows PC. How to
start it, load this project into it and drive it from the keyboard is in `BioBuzSim/MAC-AND-TEAMCODE.md`.

- In the simulator the flower arm and the camera features are switched on automatically
  (`false || InSimulator.YES` on their switch lines). On the robot they stay off until you turn them on in
  Robot Settings (steps 8.4 and 9.8).
- Use it for: does it initialize, the order of steps, timing and parking, paths around the frame, the
  autonomous menu, driver practice.
- Do not take from it: shot speeds, the shot table, flywheel gains, feed timing. Its robot numbers are
  estimates. Steps 2 to 5 below still need the real robot.

## B4. Still needs a code change and re-upload

- Gamepad button mapping (A5).
- Start marks, shot marks and parking slots (10.1).
- Which end of the field carries which four tag ids (10.2 row 7).
- The autonomous menu defaults (`AutoConfig.DEFAULT_*`). They are picked at INIT anyway.
- The hardware names in the robot configuration (`flowerArm`, `limelight`, the motors). These are changed on the Driver Station, not in code.

## B4b. Not built

- Odometry is not used to keep the camera distance when the tags are out of view (the last reading is kept 1 s).
- Fire is not blocked while the robot is moving.
- No gamepad rumble.
- The aim point is not corrected when only some of a basket's four tags are visible.
- The final autonomous position is not passed to TeleOp.
- The older moving-shot programs were left as they were (and are hidden).

---

# Part C. What to do, in order

Every step is a sequence: do the numbered actions in order. Where a value has to be found, the action says
which way to change it and when to stop ("raise by 0.05, fire again, repeat until ..."). Each section first
**tunes** its values, then **tests** them. Do not skip ahead: later steps rely on earlier ones.

| # | Section | Needs |
| --- | --- | --- |
| 1 | Build, deploy, check the settings file | robot on blocks |
| 2 | Hardware check (motor names, directions, ball path) | robot on blocks, balls |
| 3 | Flywheel: tune kS, kV, kP | robot on blocks |
| 4 | Feeding: tune the gate between shots, then test volleys | balls, a target |
| 5 | Preset speeds | balls, shooting spots |
| 6 | Jam recovery check | balls |
| 7 | Position hold: tune, then push test | open floor, a teammate robot |
| 8 | Flower arm: set up, then use | servo, a flower bed |
| 9 | Camera: set up, calibrate, tune aim and speed, then test | field tags, tape measure, balls |
| 10 | Autonomous: set up, tune, then test runs | full field |

All values are changed on the Driver Station and saved on the robot (Part A, A6). Tick the boxes as you go.

---

## 1. Build, deploy, check the settings file

- [ ] **1.1** Open the project in Android Studio and build.
- [ ] **1.2** Deploy to the robot. Put it on blocks, wheels off the ground, no balls.
- [ ] **1.3** Run `Robot Settings`. The bottom line shows `File: /sdcard/FIRST/biobuzz-settings.properties`.
- [ ] **1.4** Change any value one step and back. The bottom line says `Saved.`

## 2. Hardware check

- [ ] **2.1** Run `TeleOp_IndividualMotorToggle`. Turn on each motor alone, one at a time: `intake` (front
      roller), `windmill`, `transfer`, `shooter`, and the four drive motors.
- [ ] **2.2** For each motor write down: the part that moved, and whether it moved balls toward the shooter.
      Every motor must move the part with its own name, the right way. Fix any that do not (swap the names in
      the robot configuration, or the motor direction in code) before going on.
- [ ] **2.3** Turn the roller, transfer and windmill on forward, shooter off. Drop in ONE ball. Do this ten
      times. Write down any place a ball stops more than once: that is a mechanical dead zone (no roller touches
      the ball there) and needs a hardware fix (more squeeze, closer rollers or a guide).
- [ ] **2.4** Run `TeleOp_TestBaseCodeSpeeds`. With no ball, start feeding and read the windmill RPM. The
      windmill is a 312 RPM motor at full power: it reads close to 300. Below about 250 with no ball means it
      is rubbing or binding: fix that before step 6, or jam recovery will trigger with no jam.
- [ ] **2.5** Run `TeleOp_BaseCode`. Press G2 **Y**: the flywheel spins up, status `Spinning`, nothing else moves.
      Press G2 **RB**: it coasts down. Hold G2 **RT** (no balls): status `Starting`, then `Shooting`.

## 3. Flywheel: tune kS, kV, kP

`Shooter Tuner (kS, kV, kP)` (A6), as in the flywheel video. Gains start at 0. Robot on blocks, no balls.

**kS (just gets the wheel turning)**
- [ ] **3.1** G2 Dpad Left/Right until `>kS` is selected. Press **Y** to run.
- [ ] **3.2** Press Dpad Up once at a time. Stop at the first value where the wheel starts turning.
- [ ] **3.3** Press Y twice (stop, start). Lower kS one step at a time until the wheel no longer starts from
      stopped, then go back up one step. Write it down: ______
- [ ] **3.4** Repeat 3.3 two more times (one with the motor warm). Set kS to the average of the three.

**kV (holds the speed)**
- [ ] **3.5** Dpad Right to `>kV`. Press Y to run (kS + kV only).
- [ ] **3.6** Wait 1 s, press the **left stick button** (the kV helper). Repeat until `err` is between 0 and +20
      (speed just UNDER target).
- [ ] **3.7** Press **X** (LOW). Write down the error: ______. It will not be zero; that is normal. Press X again.

**kP (fixes drops)**
- [ ] **3.8** Dpad Right to `>kP`. Press Y to run. Press A until the step is 0.001.
- [ ] **3.9** Press X (LOW), wait 2 s, press X (HIGH), wait 2 s. Read `back in X s, overshoot Y`.
- [ ] **3.10** Raise kP one step (Dpad Up) and repeat 3.9. Keep going until the overshoot is more than 30 or
      you hear the motor hum.
- [ ] **3.11** Lower kP two steps. Press B (smaller steps) and repeat 3.9 / 3.10 once more to fine tune.
      Stop at the highest kP with overshoot under 30 and no hum.
- [ ] **3.12** Press **Start** to save (`SAVED kS .. kV .. kP ..`).

**Test**
- [ ] **3.13** Load balls. Still in the kP step, hold **RT**. Each ball's dip recovers in about the same time as 3.9:
      ______ s. Write it down.

## 4. Feeding: tune the gate between shots, then test

The gate stops feeding the moment a ball pulls the flywheel speed down, and reopens when the speed is back.
All values are in `Robot Settings` (or `Tune Everything` > TUNE). Use `TeleOp_BaseCode`, preset 1, aimed at
a target. Film in slow motion if you can.

**Tune: second ball held back**
- [ ] **4.1** Load four balls. Hold G2 RT. Watch the windmill after each ball: it must stop until the flywheel
      is back, so no ball follows straight behind another.
- [ ] **4.2** Intake > `Windmill hold rev RPM` starts at 0 (brake only). Raise it by 10 and repeat 4.1 until no
      ball follows straight behind another. Stop at the first value that works. If balls start coming back out
      of the intake, go back down by 10: that value is too high.

**Tune: every ball detected**
- [ ] **4.3** Shooter gate > `Shot drop` starts at 45. Fire four balls. The windmill must pause after EVERY ball.
      Lower `Shot drop` by 5 and fire again until it does.
- [ ] **4.4** Hold RT for 5 s with NO balls. The windmill must keep turning (no pauses). Raise `Shot drop` by 5
      and repeat until it does. Keep it above 40.

**Tune: same distance for every ball**
- [ ] **4.5** Fire four balls, three times. Mark where balls 1 and 4 land.
- [ ] **4.6** Shooter gate > `Resume below` starts at 40. Lower it by 5 and repeat 4.5 until ball 4 lands with
      ball 1. Do not go below 15.

**Tune: wait between balls**
- [ ] **4.6b** Shooter gate > `Wait between balls s` starts at 0.10 (after the speed is back, the next ball waits
      this long). Fire four balls three times. Lower it by 0.02 and repeat while every ball still lands with
      ball 1; stop at the last value that works. (Raise it by 0.02 if later balls land short.)

**Tune: speed (only after 4.1 to 4.6b pass)**
- [ ] **4.7** Intake > `Transfer hold pwr` starts at 0. Raise it to 0.15 and repeat 4.1. Keep it only if no ball
      follows straight behind another; otherwise set it back to 0.
- [ ] **4.8** Fire four balls five times. If the windmill stalls on the first ball, raise Shooter gate >
      `Windmill spinup s` by 0.05 and repeat. If it stalls on balls 2 to 4, raise `Windmill resume s` by 0.03 and
      repeat. Otherwise leave both at 0.

**Tune: intaking**
- [ ] **4.9** Hold G2 LT and drive over four balls. Each ball climbs the transfer and stays below the windmill.
      Intake > `Transfer intake pwr` starts at 0.35: lower by 0.05 until no ball is pushed past the windmill,
      then raise by 0.05 until every ball climbs. Use the middle.
- [ ] **4.10** Run `TeleOp_TestBaseCodeSpeeds`, feed, and read the transfer's peak current while it pushes a ball.
      Set Intake > `Transfer amp limit` to about 1.5 times that peak.

**Test**
- [ ] **4.11** Ten volleys of four balls. Every ball lands in the same place; no two balls leave together.
- [ ] **4.12** Time each volley from first ball to last. Worst time: ______ s (needed in step 10).
- [ ] **4.13** One volley with the battery near 12.2 V. The landing spot does not move.

## 5. Preset speeds

- [ ] **5.1** Pick your four shooting spots, near to far. Stand on spot 1, preset 1 (G2 Dpad Up/Down).
- [ ] **5.2** Fire four balls. Short: G2 Dpad Right (+25). Long: Dpad Left (−25). Repeat until 8 of 10 score.
- [ ] **5.3** Write the speed shown (`preset + trim = speed`) into Robot Settings > Shooter speeds > `Preset 1`.
- [ ] **5.4** Repeat 5.1 to 5.3 for spots 2, 3 and 4.
- [ ] **5.5** Test: restart the TeleOp, no trim, 10 shots from each spot: 8 of 10 score.

## 6. Jam recovery check

- [ ] **6.1** Load two balls tight against the windmill. Hold RT. Expected: the transfer pauses, then after 0.3 s
      the windmill and transfer back up for 0.1 s and try again, up to three times, then the screen shows
      `FEED JAMMED`. Press Back (eject) to clear.
- [ ] **6.2** Fire ten normal volleys and watch for a short backwards kick with no jam. If you see one, raise
      Intake > `Unjam after s` by 0.1 and repeat until it no longer happens.

## 7. Position hold: tune, then push test

`Tune Position Hold` (it also shows a guide on screen), or `Tune Everything` > TUNE > **Position hold**, watching
the `Hold:` line (`off` = inches from its spot, `peak` = biggest push). Open floor, 1 m clear.

**Tune**
- [ ] **7.1** Press A (hold this spot). Set `Max hold power` to 1.00.
- [ ] **7.2** Push the robot sideways firmly for 2 s and let go. Raise `Stiffness` by 0.25 and repeat until it
      starts to rock back and forth after you let go. Then lower `Stiffness` by 0.25.
- [ ] **7.3** Raise `Damping KD s` by 0.01 and repeat the push until it comes back without rocking. Stop there.
- [ ] **7.4** Lean on it steadily for 5 s. Raise `Integral KI` by 0.25 and repeat until it no longer creeps away
      during the lean.
- [ ] **7.5** Save (`Tune Position Hold`: press X; `Tune Everything` saves by itself).

**Test**
- [ ] **7.6** Hand push: peak under 3 in, back on its spot within 1 s, no rocking.
- [ ] **7.7** Robot push: a teammate drives into the side for 3 s. Peak under 4 in, back on its spot after.
      If the wheels skid while it gives way, that is the wheels' grip, not tuning (weight or grippier wheels
      are the only fix).

## 8. Flower arm: set up, then use

**How it works:** the arm is driven by a **continuous rotation servo** (it has a speed, not positions). Two **REV Touch
Sensors** stop it: `touchUp` is pressed when the arm is fully up, `touchDown` when it is fully down. Going up stops the
moment `touchUp` is pressed; going down stops at `touchDown`. It never pushes into a switch that is already pressed.
It also stops (FAULT on the screen) if:
- a move has not reached its switch after `Max travel s`;
- it is going one way and the opposite switch is pressed (direction reversed);
- both switches are pressed at once.
Resting up and sagging off `touchUp`, it drives itself back up.

**Set up**
- [ ] **8.1** Fit the **continuous rotation** servo (or a servo set to continuous mode) to a servo port. Mount `touchUp` so
      the arm presses it when fully up, and `touchDown` so it is pressed when fully down. Each switch must be pressed
      only at its end, and before the arm hits anything hard.
- [ ] **8.2** Driver Station > Configure Robot > the hub:
      Servos > that port: type **Continuous Rotation Servo**, name **`flowerArm`**.
      Digital Devices: two **REV Touch Sensor**s named **`touchUp`** and **`touchDown`**. A REV Touch Sensor
      reports on the **second** channel of its digital port (1, 3, 5 or 7), so configure it on that channel.
      Save and activate.
- [ ] **8.3** Run **`Flower Arm Test (touch sensors + tune)`**. Before Play, press each switch by hand: the screen must
      show that one `PRESSED` and the other `open`.
- [ ] **8.4** Robot Settings > Flower arm > `Enabled` = ON, then run `Flower Arm Test` again and press Play.
      Nudge the left stick up: the arm must move UP. If it moves down, press **Back** (flips `Direction`).
- [ ] **8.5** Speed is limited on purpose: defaults `Up power` 0.35, `Down power` 0.25, every move starts with a 0.3 s
      soft start, and the code never drives the servo above **0.5** whatever the settings say (`FlowerArm.MAX_SAFE_POWER`).
      **Y** (up): it stops on `touchUp`. **A** (down): it stops on `touchDown`. Neither should slam; lower
      `Up power` (Dpad Up/Down) or `Down power` (Dpad Right/Left) if it hits hard or overshoots the switch, raise it
      if it stalls. Gravity helps one way, so down usually needs less.
- [ ] **8.6** **X** = cycle test: it runs down/up/down/up and times every move. Let it do at least 10 moves with no
      FAULT. Then **Start** = SAVE. That saves the powers, the direction, `Max travel s` (slowest move x 1.5) and the
      autonomous waits Auto flower > `Arm down s` / `Arm up s` (slowest + 0.5).
- [ ] **8.7** Fault test: hold the arm by hand while it moves. It must stop with `did not reach ... in N s`.

**Use (and collect answers for step 10)**
- [ ] **8.8** In `TeleOp_BaseCode`: stop about a foot from the flower bed, facing it. G2 **A** (arm down). Hold LT
      and drive in slowly. When the balls are in, back out, G2 **A** (arm up). Do not fire while the arm moves.
- [ ] **8.9** Write down: which end goes in (front/back) ______; arm down before driving in, or once against the
      flower ______; how far back the arm needs to swing down ______ in; how long the balls take to come out
      ______ s; do balls keep coming out while backing away ______.

## 9. Camera: set up, calibrate, tune aim and speed, then test

Only after steps 3 to 5 pass. Settings are in Robot Settings groups **Camera**, **Aim**, **Aim point** and **Shot model**.

**Set up the Limelight**
- [ ] **9.1** Mount the Limelight 3A rigidly, looking toward the HIVE, nothing on the robot in view. Plug it into
      the Control Hub's blue USB 3.0 port with a short, tied-down cable.
- [ ] **9.2** Driver Station > Configure Robot > **Scan**. Name the new device **`limelight`**. Save and activate.
- [ ] **9.3** Unplug the Limelight, plug it into a laptop by USB, open `http://limelight.local:5801`.
- [ ] **9.4** Pick a pipeline slot, type **AprilTag (Fiducial)**, family **36h11**, tag size **3.25 in
      (82.5 mm)**, **Full 3D** on, no ID filter. Save. Note the slot number: ______
- [ ] **9.5** Exposure: point at a cell from your farthest spot. Raise exposure until all four tags are outlined
      steadily, then lower it until they do not blur when the robot turns. Save. Plug the Limelight back in.
- [ ] **9.6** Robot Settings > Camera > `Pipeline` = the slot number.

**Calibrate**
- [ ] **9.7** Nothing to measure by hand. Height (floor to lens, **12 3/8 in**) and left/right (**centred**, 0) are
      already in the code (`LimelightMount`). Pitch and forward are worked out in 9.10 by the calibration OpMode.
- [ ] **9.8** Robot Settings > Camera > `Camera enabled` = ON. Run `Shot Table Builder`, pick the alliance (G1 X/B).
      INIT shows `Camera: found`.
- [ ] **9.9** Point at each of our cells. The screen shows `CELL 30-33` / `34-37` (red) or `38-41` / `42-45` (blue).
      Write down which range is on which cell: ______
- [ ] **9.10** **Run `Limelight Calibration (pitch + forward)`.** Cell UP. Tape a line straight out from the point
      under the middle of the tag sticker and mark **30 in** and **78 in** (to the robot centre).
      Robot centre on 30 in, square to the cell, all 4 tags in view: press **A**. On 78 in: press **Y**.
      It shows the pitch and forward it worked out and a tag-height check (the two heights must agree within 1 in,
      otherwise redo it). Press **START** to save. (It also saves the measured height and left/right.)
- [ ] **9.11** Check: `Shot Table Builder` on the 30 in and 78 in marks reads `(tags 30)` and `(tags 78)` ± 1.
      Redo 9.10 after anything bumps the camera mount.
- [ ] **9.12** Raise and lower the cell (or use both cells): the UP one shows `UP`, the down one `DOWN`. Write
      both cluster heights from the screen: UP ______ in, DOWN ______ in. Set Auto DOWN side >
      `Basket up tag in` halfway between them.

**Tune the aim (turning), as in the auto-rotate video**
Use `Tune Everything` (G2 Start = TUNE, group **Aim**): changes apply at once.
- [ ] **9.13** Park 15 degrees to one side with the tags in view and hold G1 LT. If it turns away from the cell,
      set `Aim direction` = −1.
- [ ] **9.14** Set `Aim kD` = 0. From 20 degrees off, hold LT. Raise `Aim kP` by 0.005 and repeat until it swings
      past the target. Then lower it by 0.005 at a time until it stops on target without passing it.
- [ ] **9.15** Press B until the step is 0.0001. Raise `Aim kD` one step and repeat 9.14's turn until it arrives
      faster without passing. Stop raising as soon as it jitters, and go back one step.
- [ ] **9.16** Hold LT with the robot lined up. If it buzzes left and right, raise `Aim stop deg` by 0.25 until
      it stops buzzing.
- [ ] **9.17** Hold LT and drive around: it keeps pointing at the cell. `AIMED` within about half a second.

**Tune the speed (aim height and shot model)**
- [ ] **9.18** Measure the shooter and enter in Robot Settings > Shot model: `Launch angle deg` (ball path
      above level as it leaves, phone level on the hood), `Exit height in`, `Exit forward in` (in front of the
      robot centre).
- [ ] **9.19** Run `Shot Table Builder` with no rows (hold G2 Back + Start for 2 s to clear).
- [ ] **9.20** Near spot (closest you shoot from): hold G1 LT until `AIMED`. Fire five balls, changing the speed with
      G2 Dpad Right/Left until they drop into the middle of the opening. Press **G2 right stick button** (scale).
- [ ] **9.21** Far spot (farthest you shoot from): find the working speed the same way and compare it with
      `model` on screen. Your speed higher than model: push **G2 right stick up** once. Lower: push it **down**
      once.
- [ ] **9.22** Repeat 9.20 and 9.21 until, at both spots, the model is within 20 of the speed that works.
- [ ] **9.23** At 3 to 5 spots you really shoot from: hold LT until `AIMED`, find the speed where 8 of 10 score,
      press **G2 A** (record). A wrong row: stand there and press B.
- [ ] **9.24** Fire tolerance: Aim > `Fire tolerance deg` starts at 2.0. From a far spot, fire 10 with AIM held.
      Lower it by 0.5 and repeat until balls stop missing to the side. Do not go below 0.5.

**Test**
- [ ] **9.25** `TeleOp_BaseCode`: at each recorded spot and halfway between two, hold LT and RT: 8 of 10 score
      with no manual change.
- [ ] **9.26** Push the robot while AIM is held and the left stick is still: the screen shows
      `Aiming, spot locked`, it goes back to its spot and keeps pointing at the cell.
- [ ] **9.27** One spot again near 12.2 V: same result. (If not, go back to step 3.)

How it behaves once set up: hold AIM = turn to the UP cell's aim point, speed AUTO (model + rows), spot locked
while the left stick is still; hold FIRE = one ball at a time, only when aimed. A preset button leaves AUTO.
Camera unplugged = manual presets. Autonomous uses the camera speed only once at least one row is recorded.

---

## 10. Autonomous: set up, tune, then test runs

Programs: the 12 `<Blue|Red> <Up|Down> - <plan>` entries (A1). Settings: Robot Settings, groups starting with **Auto**.
Limelight, shot speeds and shooting range: do LIMELIGHT-AND-SHOT-TUNING.md first.

What it does: **UP side** drives to the shot mark spinning up, aims, shoots, does the flower, parks.
**DOWN side** drives to the shot mark and waits until the camera sees our cell come up, then shoots, flower if
time allows, parks (leaves for parking at about 22 to 23 s if the cell never comes up). It always leaves in time to
be parked by 29.5 s; everything stops at 29.7 s. Shooting ends at four counted balls, 2.5 s of feeding with no
ball, or when the shooting time is used up.

**Set up**
- [ ] **10.1** Start: **back of the robot touching the wall**, front facing the cell, in the wall tile in front of
      the cell. All three plans on one side use the same spot. Map: [auton-start-positions.png](auton-start-positions.png).
      - **Up side: middle of the tile** (2.8 in to each side seam).
      - **Down side: corner of the tile next to the centre line**: robot side on the seam at the centre line. The
        flower pot sits in the tile's other corner, and the middle of the tile would overlap it.
      INIT tells the odometry the robot is exactly there; the INIT Limelight check flags a robot that is off.
      Measure Auto > `ROBOT_BACK_TO_CENTER_IN` (back face to robot centre, default 9) if the robot is not 18 in long.

| Start | Alliance | x (m) | y (m) | Faces |
| --- | --- | --- | --- | --- |
| Up side, middle of tile | Red | -0.300 | -1.568 | 90 deg (toward the cell) |
| Up side, middle of tile | Blue | +0.300 | +1.568 | 270 deg |
| Down side, tile corner | Red | -0.229 | +1.568 | 270 deg |
| Down side, tile corner | Blue | +0.229 | -1.568 | 90 deg |

Worked out in code (`SideRoute.startTile`): the tile column holding the middle of the cell's tag row (Auto camera >
`Tag centre x m`), first tile from the wall; the middle if it keeps `START_POT_CLEARANCE_IN` (1 in) from every flower
pot, otherwise the tile corner that does.

Shot marks and parking slots (red; blue is turned half a turn): Basket 1 start shoots from (-0.349, -1.099)
facing about 88 deg and parks at (-1.451, +0.640); Basket 2 start shoots from (-0.349, +1.099) facing about
272 deg and parks at (-1.451, +1.257). Start marks, shot marks and parking slots are in code
(`CycleRoute`, `DelayedBasketTwoRoute` in `Auton/cycles/`) and need a rebuild to change.

- [ ] **10.2** Set each assumption in Robot Settings (bold = Robot Settings, no rebuild; others are code):

| # | Assumption (default) | Set it from | Where |
| --- | --- | --- | --- |
| 1 | UP side = Basket 1 wall mark, DOWN side = Basket 2 wall mark | which cell is up at the start of a real match | **Auto assumptions > Swap start walls** |
| 2 | Flower 1 near the Basket 1 wall, flower 2 on the Basket 2 wall | the field | **Auto assumptions > Flower 1 / Flower 2 x, y, facing** |
| 3 | Basket 1 start uses flower 1, Basket 2 start uses flower 2 | your strategy | **Auto assumptions > Flower for B1 / B2 start** |
| 4 | Front (intake end) goes into the flower | step 8.7 | **Auto assumptions > Flower back first** |
| 5 | Arm comes down before driving in | step 8.7 | **Auto assumptions > Arm down before in** |
| 6 | Intake runs driving in, at the flower and backing out | step 8.7 | **Auto assumptions > Intake driving in / backing out / extra s** |
| 7 | The Basket 2 end carries the lower four tag ids | step 9.9 | **Camera > Blue / Red first tag**; ends swapped = code change in `SideRoute.basketFirstTag` |
| 8 | Each start parks in the half nearer its own wall | agree with your partner | code: `CycleRoute.OUR_PARK_Y_METERS`, `DelayedBasketTwoRoute.park` |
| 9 | Four balls loaded | match rules | **Auto shooting > Balls** |
| 10 | Flywheel spins during the whole DOWN-side wait | battery after a few runs | **Auto assumptions > Spin while waiting** |

**Tune**
- [ ] **10.3** In TeleOp, stand on each shot mark and find the speed that scores (as in step 5). Set Auto shooting >
      `UP shot speed` and `DOWN shot speed`.
- [ ] **10.4** Set Auto shooting > `Shoot budget s` to your worst volley time from step 4.12 plus 1.5 s.
- [ ] **10.5** Set Auto flower > `Standoff in` and `Collect s` from step 8.7.

**Test runs (each one: centre the robot on its tile, INIT, check `LIMELIGHT CHECK: OK`, Play)**
- [ ] **10.6** Plan "Park only", both sides, both alliances. The robot goes round the frame and stops in its slot,
      leaving the other half free. `Parked at` under 29.5 s.
- [ ] **10.7** Camera OFF (Auto camera > `Use camera` OFF), UP side, "Shoot + park": four balls score,
      `Balls counted` reaches 4, robot parks. DOWN side: it shoots at 15 s (tip the cell by hand before then).
- [ ] **10.8** Camera ON, UP side: the robot turns slightly to line up, then shoots. If it turns away and shoots
      after 2.5 s from the planned heading, set Auto camera > `Turn sign` = −1 and repeat.
- [ ] **10.9** DOWN side: tip the cell by hand at about 8 s; shooting starts about 0.7 s later. Run again without
      tipping: it leaves for parking at about 22 to 23 s without shooting.
- [ ] **10.10** UP side, "Shoot + flower + park": stand-off point, arm, drive in, intake, back out, arm up, park.
      If it bounces off the flower bed, lower Auto flower > `Push m per s` by 0.05 and repeat.
- [ ] **10.11** Each plan you will use, five times in a row: `Parked at` under 29.5 s every time.

**Parking next to a parked teammate (always assumed)**
- Up-side robot parks in the near half of the parking zone, Down-side robot in the far half. When we park, the code
  assumes the teammate is **already parked** in the other half: no part of our robot (rotated corners included) may
  enter their half, on any path, from anywhere (`TeammateSpace`).
- Every parking path ends 12 in straight out from our slot on the field side, then the robot **slides slowly
  straight into the slot towards the wall** (Auto timing > `PARK_ENTRY_IN`, `PARK_SLIDE_M_PER_S`), never towards the teammate.
- If no path keeps clear of the teammate (only possible if the robot ends up on the wrong side of the frame), it
  **holds where it is** (`BLOCKED`, LEDs chase) rather than push, and checks again every second.
- `tools/auton/run-field-checks.ps1` checks **nothing hits any part of the field**: the 18-inch robot at its real
  heading, along every path of all 12 programs and every fallback path (parking, flower, approach and re-centring from
  pushed / misplaced poses, backing out of the frame), against the walls, the metal frame, all four flower pots (only the
  flower step may touch its own pot), the other alliance's half, and while parking the teammate's half. About 200,000
  robot poses; must say PASS. Run it after any route or setting change.
- `tools/auton/run-parking-checks.ps1` tries parking from about 2,500 positions (every point on every path, plus
  pushes up to 6 in / 25 deg): every one must find a frame-clear, teammate-clear path. Run it after any route change.
- [ ] **10.11d** Park a second robot (or a box 18 x 18 in) in the teammate's slot. Run every plan: our robot never touches it.

**Flower arm safety**
- The robot only moves with the arm down on the straight push into the flower and the straight back-out.
- It waits for `touchDown` before driving into the flower. If the arm does not get there within `Arm down s`, it skips
  the flower and parks.
- Whenever it leaves for parking with the arm down (flower finished, out of time, or an error), it first slides straight
  back out to the stand-off point, **then** raises the arm and stands still **until `touchUp` is pressed**, **then**
  drives. If `touchUp` is not pressed within `Arm up s` it tries once more, then parks anyway (it must park), with a
  warning on the screen. The parking time budget includes this.
- The arm's safety loop keeps running after the auton's own work is done, so the servo always stops at its switch.
- INIT shows the arm and both switches. The arm must be UP (`touchUp` pressed) before the match, or the LEDs flash gold.
- If the clock runs out while it is still in the flower bed, the arm is left down (swinging it up there could hit the flower).
- [ ] **10.11e** During "Shoot + Flower + Park", watch the arm: it stops on `touchDown` before driving in, and on
  `touchUp` before the robot drives away from the stand-off point. (`Arm down s` / `Arm up s` come from step 8.6.)

**Recovery tests (the auton never stops early; the only ends are `Parked` and the 29.7 s clock)**

| Problem | What the robot does | Test |
| --- | --- | --- |
| Pushed off the shot mark while shooting | After `Re-centre after s` (0.75 s) drives back onto the mark; after 3 tries accepts the looser `Relaxed on-mark in` window and shoots anyway | **10.11a** push it 4 in while shooting: `re-centring` shows, it goes back and keeps shooting |
| Leg blocked or ends short | After `LEG_LATE_SEC` (4 s) and standing still, re-drives the rest of the leg from where it is | **10.11b** hold the robot 2 s on the way to the shot mark, let go: `re-driving`, then it carries on |
| Inside the frame keep-out | Drives straight out to the nearest clear spot, then re-plans the step | (happens only after a big push or odometry drift) |
| Odometry glitch | Wheels stop and it waits; when readings are good again it re-plans the step and carries on | **10.11c** pull the Pinpoint cable for 1 s: `ODOMETRY GLITCH`, then it carries on |
| Code error | Shows `Recovered errors`, re-plans the step; after 3 errors parks by the plainest path | (should not happen) |
| Running out of time | THE PARKING RULE leaves for parking in time, whatever it is doing | 10.9 (Down side, never tip the cell) |

**Optional: camera position fix (corrects the odometry at the shot mark)**
- [ ] **10.12** Measure on the real field, cell UP, red side: x of the middle of the tag row from field centre,
      the row's distance from the centre line, and each tag's offset along the row. Enter them in Auto camera >
      `Tag centre x m`, `Tag row y m`, `Tag 1-4 offset in`.
- [ ] **10.13** With `Camera fix position` OFF, place the robot carefully and run UP side "Shoot + park" three
      times. Write down `Camera position check` (x, y) each run: ______ / ______ / ______. They should be
      within 1 in and the same each run. Large but the same: recheck 9.10, 9.11, 10.1 and 10.12 in that order.
      Different each run: recheck the mount and 9.5.
- [ ] **10.14** Place the robot 2 in off its mark: the check shows about 2 in. Turn `Camera fix position` ON and
      run again with a hand near stop: the line ends `APPLIED` and the robot parks where it does when placed
      correctly. If it lurches when `APPLIED` appears, turn the switch off.
