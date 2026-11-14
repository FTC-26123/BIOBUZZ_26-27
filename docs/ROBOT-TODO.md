# Robot to-do: flower ramp, LEDs, Limelight, autonomous

One checklist, in order. Tick each box as it is done and fill in the blanks. Each part says which earlier part it needs.
More detail: [ROBOT-SETUP.md](ROBOT-SETUP.md) (step numbers in brackets) and
[LIMELIGHT-AND-SHOT-TUNING.md](LIMELIGHT-AND-SHOT-TUNING.md). Almost every number lives in the **Robot Settings** OpMode
(saved on the Control Hub, no rebuild).

---

## 1. Robot configuration (Driver Station > Configure Robot)

- [ ] Flower ramp servo on a servo port: **Continuous Rotation Servo**, name **`flowerArm`**.
- [ ] Two digital ports: **REV Touch Sensor** named **`touchUp`** and **`touchDown`**. A REV Touch Sensor reads on the
      port's **second** channel (1, 3, 5 or 7); if one never shows PRESSED later, try the other channel.
- [ ] Blinkin on a servo port: **REV Blinkin LED Driver**, name **`led`**.
- [ ] Limelight 3A in the Control Hub's blue USB 3.0 port, then **Scan**: name **`limelight`**.
- [ ] Save and activate.

---

## 2. Flower ramp (continuous servo + touch sensors) (step 8)

How it works: the servo runs until a switch is pressed. `touchUp` = fully up, `touchDown` = fully down. It is slow on purpose:
up 0.35, down 0.25, a 0.3 s soft start, and the code never goes above **0.5** power. It stops with a FAULT if:
- a move takes longer than `Max travel s`;
- the opposite switch gets pressed (direction backwards);
- both switches are pressed at once.
Resting up and sagging off `touchUp`, it drives itself back up.

**Mount**
- [ ] `touchUp` is pressed only when the arm is fully up, **before** the arm hits anything hard.
- [ ] `touchDown` the same for fully down.

**Test and tune: run `Flower Arm Test (touch sensors + tune)`**
- [ ] Before Play: press each switch by hand → only that one shows `PRESSED`.
- [ ] Robot Settings > Flower arm > `Enabled` = ON. Run the test again, press Play.
- [ ] Nudge the left stick up → the arm moves **up**. If it moves down, press **Back** (flips direction).
- [ ] **Y** = up: stops on `touchUp`. **A** = down: stops on `touchDown`. **B** = stop.
- [ ] Hits the switches too hard → lower `Up power` (Dpad Up/Down) or `Down power` (Dpad Right/Left). Stalls → raise it.
      Final: up ____ down ____ (max 0.5).
- [ ] **X** = cycle test, at least 10 moves with no FAULT. Slowest up ____ s, slowest down ____ s.
- [ ] **Start** = SAVE. This saves the powers, the direction, `Max travel s` and the auton waits `Arm up s` / `Arm down s`.
- [ ] Fault test: hold the arm while it moves → it stops with `did not reach ... in N s`.
- [ ] Direction test: press Back, press Y → FAULT `DIRECTION is reversed?`. Press Back again to fix it, then save.
- [ ] Sag test: arm up, push it off `touchUp` by hand → it drives itself back up.
- [ ] Only if the arm sags off a switch on its own: set `Hold power` ____ (otherwise leave 0).

**In TeleOp** (`Blue TeleOp` / `Red TeleOp`)
- [ ] G2 **A** toggles down / up and it stops on each switch. Don't fire while it moves.
- [ ] Practise the flower bed by hand (step 8.8). Stop about a foot away, A (down), hold LT, drive in slowly, back out, A (up).
- [ ] Write down: which end goes in ____; arm down before driving in, or once against the flower ____;
      how far back the arm needs to swing down (`Standoff in`) ____ in; how long the balls take to come out ____ s;
      do balls keep coming out while backing away ____.

---

## 3. LEDs (Blinkin): first check

- [ ] Robot Settings > LEDs > `LEDs enabled` = ON. The INIT screen line `LEDs` shows a pattern, not "not found".
- [ ] INIT of a Blue program → blue breathing. Red program → red breathing.
- [ ] Unplug the Limelight, INIT → breathing alternates with **flashing gold**. Plug it back in.

(The full colour checks are in parts 7 and 8, once everything else works.)

---

## 4. Limelight (step 9)

Already in the code: lens height **12 3/8 in**, camera **centred** left/right.

**Pipeline** (laptop, Limelight plugged in by USB, `http://limelight.local:5801`)
- [ ] AprilTag pipeline: family **36h11**, tag size **3.25 in**, **Full 3D** on, no ID filter. Slot ____
- [ ] Exposure from your farthest shooting spot: all 4 tags outlined, no blur when the robot turns. Plug the camera back into the hub.
- [ ] Robot Settings > Camera > `Pipeline` = the slot, `Camera enabled` = ON.

**Calibrate (nothing to measure by hand)**
- [ ] Cell UP. Tape a line out from under the **middle** of the tag sticker. Mark **30 in** and **78 in**, measured to the robot centre.
- [ ] Run **`Limelight Calibration (pitch + forward)`**. Robot centre on 30 in, square to the cell, all 4 tags in
      view → **A**. On 78 in → **Y**.
- [ ] The two tag heights on screen agree within 1 in (otherwise redo).
- [ ] **START** = save. Pitch ____ deg, Forward ____ in.
- [ ] Check in `Shot Table Builder`: it reads `tags 30` and `tags 78` (± 1) on the marks.
- [ ] Which tag ids are on which cell (`CELL 30-33` etc.): ____
- [ ] UP cluster height ____ in, DOWN ____ in. Auto DOWN side > `Basket up tag in` = halfway: ____

**Aim** (`Tune Everything`, Start = TUNE mode, group **Aim**)
- [ ] Hold G1 LT 15 deg off: it turns **toward** the cell (else `Aim direction` = −1).
- [ ] `Aim kD` = 0. Raise `Aim kP` until it swings past, then back off until it stops on target: ____
- [ ] Raise `Aim kD` in tiny steps until it arrives faster without overshoot. Stop at any jitter: ____
- [ ] Buzzes left/right when lined up → raise `Aim stop deg`: ____
- [ ] Hold LT and drive around: it keeps pointing at the cell, `AIMED` within about 0.5 s.

---

## 5. Shot speed and shooting range (`Shot Table Builder`, needs part 4)

- [ ] Shot model entered: `Launch angle deg` ____, `Exit height in` ____, `Exit forward in` ____ (step 9.18).
- [ ] Clear rows (G2 Back + Start, 2 s). Scale the model at a near and a far spot until it is within 20 of what works (9.20–9.22).
- [ ] At 5 spots about 12 in apart, including both auton shot marks: hold LT until `AIMED`, then:
      - fire 5 per speed, going up 25 at a time;
      - note the lowest and highest speed that score 4 of 5;
      - set the middle of that range and confirm 8 of 10;
      - **G2 A** to record the row.

| Spot | `Dist` (in) | Low | High | Recorded | 8/10? |
| --- | --- | --- | --- | --- | --- |
| | | | | | |
| | | | | | |
| | | | | | |
| Up shot mark | | | | | |
| Down shot mark | | | | | |

- [ ] **Closest:** go 6 in closer at a time until nothing scores 8/10, then halve back. Exact minimum ____ in. Record a row there.
- [ ] **Farthest:** same going back. Exact maximum ____ in. Record a row there.
- [ ] Aim > `Min shot dist in` = minimum + 2 = ____. `Max shot dist in` = maximum − 2 = ____.
- [ ] Halfway between two rows: 8/10 with no manual change. Walk from too close to too far with AIM held: the in-range
      LEDs only show between min and max.
- [ ] One spot again near 12.2 V: still scores.
- [ ] `Fire tolerance deg`: from a far spot, lower by 0.5 until balls stop missing sideways (not below 0.5): ____

---

## 6. Autonomous: set up (step 10, needs parts 2, 4, 5)

The 12 programs are named `<Blue|Red> <Up|Down> - <Shoot + Flower + Park | Shoot + Park | Park Only>`. Each preselects the matching TeleOp.

**Start position** (map: [auton-start-positions.png](auton-start-positions.png)): **back of the robot touching the wall**,
front facing the cell, in the wall tile in front of the cell. All three plans on a side use the same spot.
- **Up side:** middle of the tile (2.8 in to each side seam).
- **Down side:** corner of the tile next to the centre line. The robot's side is on that seam; the flower pot is in the other corner.

- [ ] Robot length: back face to robot centre = ____ in. If not 9, set Auto > `ROBOT_BACK_TO_CENTER_IN`.
- [ ] Assumptions in Robot Settings > **Auto assumptions** (step 10.2):
      which wall is UP ____; flower positions ____; which flower each start uses ____;
      which end goes in ____; arm down before / at the flower ____; intake timing ____; spin while waiting ____.
- [ ] Auto shooting > `UP shot speed` ____ and `DOWN shot speed` ____ (from part 5). `Balls` = 4.
- [ ] Auto shooting > `Shoot budget s` = worst volley time + 1.5 = ____.
- [ ] Auto flower > `Standoff in` ____ and `Collect s` ____ (from part 2).
- [ ] Every program's INIT screen shows **`LIMELIGHT CHECK: OK`** and the flower arm **UP**. The Up side expects about **49 in** to the cell.

---

## 7. Autonomous: test runs

Each run: robot on its start, INIT, check `LIMELIGHT CHECK: OK` and the arm UP, load 4 balls, Play.

**Basic runs**
- [ ] **Park Only**, both sides, both alliances: drives to the slot, slides slowly into it, `Parked at` under 29.5 s.
- [ ] Camera OFF (Auto camera > `Use camera` OFF), Up "Shoot + Park": 4 balls score, `Balls counted` = 4, parks.
- [ ] Camera OFF, Down "Shoot + Park": shoots at 15 s (tip the cell by hand before then).
- [ ] Camera ON, Up: turns slightly to line up, then shoots. If it turns away → Auto camera > `Turn sign` = −1.
- [ ] Down: tip the cell by hand at about 8 s → shooting starts about 0.7 s later.
- [ ] Down, never tip the cell → leaves for parking at about 22–23 s without shooting.
- [ ] Wrong-program check: Down program with the cell UP → INIT says to pick the other program, LEDs flash gold.

**Flower** ("Shoot + Flower + Park")
- [ ] Stand-off point → arm stops on `touchDown` → drives in slowly → collects → backs straight out → arm stops on `touchUp`
      → only then drives to park.
- [ ] Bounces off the flower bed → lower Auto flower > `Push m per s` by 0.05.
- [ ] Unplug `touchDown` before the run → it skips the flower and parks.
- [ ] Hold the arm so it can't get up → it tries again, then parks anyway with a warning.

**Parking next to the teammate**
- [ ] Put a box 18 × 18 in (or the partner robot) in the teammate's half of the parking zone. Run every plan: our robot never touches it.
- [ ] The last bit into the slot is a slow, straight slide toward the wall.

**Recovery** (the auton never stops early; it only ends `Parked` or at the clock)
- [ ] Push it 4 in while it shoots → `re-centring`, it goes back and keeps shooting.
- [ ] Hold it 2 s on the way to the shot mark → `re-driving`, then it carries on.
- [ ] Pull the Pinpoint cable for 1 s → `ODOMETRY GLITCH`, wheels stop, then it carries on.

**Timing and LEDs in auton**
- [ ] Chase while driving, heartbeat while the Down side waits, solid while lining up, **green** while firing,
      **violet** during the flower step.
- [ ] **20 s: fast white strobe** for 2 s (10 s left).
- [ ] Rainbow when finished (flashing gold if something went wrong during the run).
- [ ] The program **ends itself at 30 s**.
- [ ] After auton the Driver Station has the right TeleOp selected (Blue / Red TeleOp).

**Repeatability**
- [ ] Each plan you will use, **5 times in a row**: `Parked at` under 29.5 s every time. Plans: ____

**Optional: camera position fix** (step 10.12–10.14). Leave `Camera fix position` OFF until 3 runs show the same small numbers.

---

## 8. TeleOp: LEDs and timing (needs parts 2–5)

- [ ] Flashing gold when the feed jams (eject with Back).
- [ ] **Green** = fire now (in range, aimed, at speed). Solid alliance = in range. Heartbeat = basket seen, not in range.
- [ ] Yellow = spinning up, lime = at speed (no basket in view).
- [ ] Violet = flower arm down. Dim alliance breathing = idle.
- [ ] **90 s: slow white pulse** (30 s left). **110 s: fast white strobe** (10 s left).
- [ ] The program **ends itself at 120 s**. For long practice: Robot Settings > Match > `TeleOp length s` = 0 (set it back after).
- [ ] All colours readable from where the drive team stands, in bright light.

---

## 8b. Robot spins in place at the start of TeleOp: find the cause

Already changed in code:
- The position hold no longer arms at Play. It waits until the driver has driven once.
- A runaway turn now stops at 30° instead of 120°. After two runaways the hold switches itself off for the rest of
  the match, and the screen says `OFF: heading kept running away`.
- Every stick has its own dead zone (0.08), so a resting or drifting stick can't turn the robot.

To find the real cause:
- [ ] Run **`OLD One Controller (plain drive, no hold)`** (group Test). It's the one-controller TeleOp from before the
      position hold was added: plain driving, no hold, no camera. Spins here too → not the hold: go to the stick test.
      Doesn't spin here → it was the hold: go to the heading test.
- [ ] **Stick test:** in any TeleOp with the sticks untouched, read the gamepad values on the Driver Station (gamepad
      icon). `right stick x` should sit at 0. Anything above about 0.08 is a worn or uncalibrated stick: re-plug or
      replace the controller.
- [ ] **Heading test:** run `TeleOp_OdometryTesting`. Turn the robot by hand to the **left** (counter-clockwise from
      above): `Heading` must go **up**. If it goes down, the Pinpoint heading is reversed. Then the hold, and the auton
      turns, push the wrong way. Fix the Pinpoint setup in `pedro/Constants.java`.
- [ ] In the real TeleOp, read `Position hold` on the screen when it happens: `Turned NN deg off` or `OFF: heading kept
      running away` = the heading test above.
- [ ] Holding **G1 right stick button** bypasses the hold. If holding it stops the spin, it is the hold.

## 9. Before every match

- [ ] Right program: alliance, Up/Down side, plan.
- [ ] Robot on its start: back touching the wall, middle (Up) / corner (Down) of the tile, square.
- [ ] 4 balls loaded. Flower arm UP.
- [ ] INIT: `LIMELIGHT CHECK: OK`, flower arm `ready, UP`, LEDs breathing (no gold).
- [ ] `TeleOp length s` = 120 (not 0 from practice).

---

## 10. Open decisions and known gaps

- [ ] **Down start touches the centre line** (it's in the tile corner, to clear the flower pot). Is that allowed by the game
      rules? If not, a different start tile is needed.
- [ ] LEDs: TeleOp doesn't show a flower-arm FAULT, only text on the Driver Station screen.
- [ ] LEDs: violet means only "commanded down". You can't tell moving, stuck halfway, and confirmed down apart.
- [ ] LEDs: TeleOp INIT doesn't check the flower arm is up (auton INIT does).
- [ ] LEDs: auton "blocked, holding clear of the teammate" looks the same as driving (chase).
- [ ] LEDs: auton ends in flashing gold after **any** glitch, even if it recovered and parked fine.
- [ ] Flower: the path checks don't test whether the arm clears the flower bed when it swings at the stand-off point.
- [ ] After any route or setting change, `tools/auton/run-field-checks.ps1` and `tools/auton/run-parking-checks.ps1` must say PASS.
- [ ] Commit the code.
