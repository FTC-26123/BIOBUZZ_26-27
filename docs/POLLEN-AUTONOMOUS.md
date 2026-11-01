# Pollen cycle autonomous



## Driver Station programs



- **26123 Red Rush - pollen cycles** (`RedRush`)

- **26123 Blue Rush - pollen cycles** (`BlueRush`), mirrored field route
- **26123 Red No Flowers** (`RedNoFlowers`)
- **26123 Blue No Flowers** (`BlueNoFlowers`), mirrored field route



These are the current two-tip simulator baseline, ported from BIOBUZZ-sim main commit `9c28f9e429bb098fabf1819c1376b188fa3368f6`. They prioritize flower pollen, all twelve alliance-side preplaced pollen balls, and useful shooting transfers toward the next pickup. They can attempt further cycles and a partial teleop pickup while reserving time to park. Nectar is unsupported. Third/fourth tips are not verified. Neither program has been validated on the physical robot.



These four autonomous wrappers are registered. Older flower/adaptive wrappers failed a fresh two-tip qualification (zero or one completed tip and a far-fire timeout), and nectar-dependent tip wrappers are excluded. Their shared route definitions remain internal; they are not additional Driver Station programs.



## Required setup before field testing



1. The team-provided tuned `ForesightConfig` is now included in `Constants`, with Pinpoint offsets x = -5.161768995870755 in and y = -3.5600169624869284 in. The follower factory is fully configured; there is no missing-tuning initialization block. The supplied lateral controller is retained exactly, using the primary lateral gain in both piecewise ranges as confirmed by the user. Drivetrain directions now match the active MecanumDrive subsystem (left REVERSE, right FORWARD); Pinpoint offsets are preserved and still need physical verification.

2. Configure the required hardware names: `frontLeftMotor`, `frontRightMotor`, `backLeftMotor`, `backRightMotor`, `pinpoint`, `shooter`, `intake`, `windmill`, `gate`, `limelight`, `Webcam 1` (Logitech USB camera). Gate closed/open positions are 0.50/0.27. Match the existing subsystem conventions: shooter FORWARD with positive encoder velocity; intake REVERSE with negative intake power; windmill FORWARD with positive power. Verify these with the robot raised before loading balls. Shooter uses RUN_USING_ENCODER; intake and windmill use RUN_WITHOUT_ENCODER with BRAKE.

3. Configure **Limelight neural detector pipeline 7**, using the trained model with the exact case-sensitive class label **`yellow_pollen`**. All other classes (including nectar) are ignored. This code reads `getDetectorResults()`, not color results. The model and its labels must already be uploaded/configured in the Limelight web UI; this repository does not train or install the model. Limelight stays on 7 throughout autonomous.

4. Connect the Logitech USB webcam and name it **`Webcam 1`** in Robot Configuration. FTC SDK VisionPortal/AprilTagProcessor detects tag IDs independently of Limelight, with the SDK default camera resolution. Red near/far tag groups are 34–37/30–33; blue near/far are 42–45/38–41. Confirm these IDs and camera streaming on your field. Fire requires an expected-cell **single tag** acquired in the last 180 ms; missing, stale, future-dated or wrong-cell detections block feeding. No tag pose, lens intrinsics or field localization from the webcam is used for shooting; Pinpoint remains the pose source. Aim the webcam so hive tags remain visible throughout shooting transfers. Its mount can differ from Limelight's. Verify your exact Logitech model and supported stream settings on hardware; the code does not depend on a model-name match.

5. Run **26123 Dual Camera Check** before autonomous. It reports accepted pollen count and expected-cell tag checks without commanding motors or needing the Pedro tuning configuration. Stop closes both camera sessions.

6. Measure the Limelight camera mount. Ground projection currently assumes height **0.305 m**, pitch **+10 degrees**, zero yaw/roll and zero horizontal offset relative to robot pose. Physically match these assumptions or update the projection; this port does not move/configure a simulated camera.

7. Measure shooting and feeder behavior. Current estimates assume 6000 RPM motors, 28 encoder ticks/rev, a 72 mm roller, 0.6285866147 m feed path, 1:1 reduction, 25% transfer, 35 ms gate delay, and 350 ms minimum spacing. The flywheel forecast, velocity-loop response, load-event threshold, ball drag/backspin, launch slip and gate pulse timing require real tests and tuning. A flywheel load event is a feed estimate, not a scoring sensor. Do not treat simulator gains/shot percentages as hardware calibration.

8. Verify the muzzle geometry: forward 0.059119 m, right approximately zero, height 0.288940 m, launch angle 55 degrees. The chassis envelope is approximately 0.401 by 0.458 m. Path reference limits are 1.0 m/s and 4.0 m/s²; these are requested limits, not measured 435 RPM / 104 mm drivetrain capabilities. Tune actual acceleration, braking and tracking before fast shots.

9. Start with four pollen preloads and an otherwise reset field. Coordinates are field-centered inches, +X right, +Y up; Pedro heading zero points +X. Red starts at (-0.360, -1.564259) m, heading 90 degrees; blue is its 180-degree mirror, (+0.360, +1.564259) m, heading 270 degrees. Init displays the placement. Verify localizer heading and alliance-half clearance on the real field.



## Motor wiring checked against the team repository

| Hardware name | SDK direction | Inward / launch command |
| --- | --- | --- |
| `intake` | REVERSE | -0.8 power |
| `windmill` (transfer) | FORWARD | +1.0 power |
| `shooter` | FORWARD | positive ticks/second |
| `frontLeftMotor`, `backLeftMotor` | REVERSE | positive power drives forward |
| `frontRightMotor`, `backRightMotor` | FORWARD | positive power drives forward |

These match the active `subsystems/Intake.java`, `Shooter.java` and `MecanumDrive.java` in FTC-26123/BIOBUZZ_26-27 master. The older Pedro constants reversed both right motors; this revision changes them to FORWARD to match the active drivetrain. Direction and power sign are separate: the reversed intake needs a **negative logical command** to pull inward. `Intake.switchSpeed(+speed)` now uses that same inward convention and reverses both mechanisms for a negative speed.

`windmill` is required, not optional. Autonomous initialization configures the shooter for encoder velocity control, the intake/transfer for power control with BRAKE, stops all three motors and closes the gate. Stop and completed feed pulses stop both intake and transfer.

Use **26123 Motor Direction Check** with the robot supported and mechanisms clear. It needs neither cameras nor Pedro tuning. After Start, hold **right bumper** plus one control: **A** runs intake at -0.25; **B** runs windmill at +0.25; **X** commands shooter at +600 ticks/s; **D-pad up** runs all drive wheels at +0.15. Only one check runs at a time. Gate remains closed; releasing the bumper or pressing Stop clears commands. Verify encoder readings and physical direction before an autonomous field run. Software tests cannot confirm wiring or real motor motion.

## Finite flower pickup and no-flower routes

Rush flower stages now hold the pickup pose while an encoder-based transport budget completes. Each ball requires estimated roller travel, at least 0.18 s engagement, and stack-settle allowance (0.13 s in the controller). Pausing/lost contact resets the contact budget; a stalled encoder cannot finish it. This estimates acquisition time, not actual inventory; measure the draw/slip/contact times on hardware. The simulator separately moves each ball through a 0.10 m draw and lets the remaining column settle under gravity, with 0.18 s minimum engagement per ball. It no longer removes a whole stack in a few simulation steps. These are adjustable reduced-order estimates, not a detailed pot/roller contact model.

RedNoFlowers and BlueNoFlowers use **exactly the corresponding Rush starting position and heading**. They shoot preloads while approaching the garden, collect garden pollen, score toward the far cell, clean up the remaining garden pollen, recover first-tip spill from outside the metal assembly, and optionally recover far-end loose pollen before scoring toward the park. They never run a flower action. The detector collector excludes poses within 0.42 m of known pot centers, since a yellow_pollen detection alone cannot identify whether a ball is loose or inside a pot. All collection remains pollen only.

The short-lived spill map contains only camera-projected targets, expires after 3 s and still requires fresh camera frames; a clear path is checked before each queued approach. It avoids repeated return-to-scan trips between nearby visible balls. Shot bias on the initial no-flower batch favors the alliance-side portion of the hive. Scatter, retention and recovery vary, so no fixed two-tip no-flower result or mathematically optimal route is promised.

## Whole metal-frame clearance correction

The no-flower routines no longer use the inner scan corridor or drive between the support feet. The entire central assembly is a forbidden rectangle with half-extents 0.67 m by 0.52 m. Planning expands it by a 0.34 m swept robot radius and 0.03 m tracking allowance, yielding a center keepout of +/-1.04 m in x and +/-0.89 m in y. This is a conservative envelope for the current approximately 0.401 x 0.458 m chassis including its body offset, not a claim that arbitrary robot sizes fit.

A visibility-graph planner uses alliance-side outer corners (-1.13, +/-1.05) m (mirrored for blue). Every ordinary move, rolling-shot transfer, camera pickup detour and parking leg uses the same rule. Straight lines are permitted only when the entire segment clears the expanded assembly. The pickup scan remains outside at red (-1.13, -0.78) m, Pedro heading 0 degrees, with the corresponding 180-degree blue mirror. Loose pollen near an end is approached from the safe end side; goals inside the keepout are rejected. No camera target can authorize an under-frame shortcut. A runtime interlock stops outputs if measured robot position enters the conservative center keepout.

Earlier no-flower performance measurements that used the inner corridor do not validate clearance and are superseded. The latest runs check the actual rotated chassis and interpolated sweeps against the whole metal envelope independently of the waypoint planner. These are simulated geometric checks; confirm field/chassis dimensions and real tracking margin before hardware use.

## Behavior and limits



The controller estimates release pose from odometry, feeder travel and flywheel recovery. A drag/backspin trajectory solver compensates for robot translation and rotation. Scoring paths move one way toward useful pickups, with a frame-support bypass. Fast-shot permission requires current speed over 0.40 m/s and predicted release speed over 0.75 m/s, a valid trajectory, expected fresh USB-webcam tag, and chassis clearance. If unsafe while moving, the remaining shot can finish at the pickup approach; it does not add artificial shooting laps.



Tipping is not read from simulator truth. A 3.15 s flight/hinge allowance follows relevant feed stages; collection uses projected pollen detections. Parking uses the measured pose and a reserved return budget, with a hard 29.7 s cutoff. A timeout can terminate early or trigger parking; a stopped drivetrain is not proof of parking. Stop closes the gate and stops mechanisms, follower and camera.



The shared controller includes older route branches for compatibility; the robot repo enables Rush and No Flowers. It imports no simulator hardware stubs, field truth or `SimPedro` tuning. It keeps the team's Java 8 build configuration, replacing source records/type inference with ordinary immutable classes and explicit types.



## Validation



Historical benchmark before finite flower extraction (superseded for current timing): a matching simulator run on the source revision recorded RedRush 16/16 shots, two completed tips, three fast shots and parking at 28.43 s; BlueRush recorded 17/17, two tips, three fast shots and parking at 28.41 s. Timing/scatter varies between runs. These results describe the simulation before the hardware-direction and follower integration changes; they do not certify the port on a robot.



Historical two-camera trials before finite flower extraction scored RedRush 18/18 and BlueRush 17/17; both completed two tips and parked. These do not certify the real robot.

The camera source changes are covered by shared label/age/ID contract checks and simulator dual-camera lifecycle/image tests; physical webcam and neural-model performance remains unmeasured. Legacy `red_forward`, `red_cycle`, `HiveAim` and TeleOp alignment code retain their original vision interfaces; the new split applies to the cycle autonomous family and Dual Camera Check.



Build the real Android project with `gradlew.bat :TeamCode:assembleDebug`. Physical calibration, feed confirmation, camera alignment, drivetrain tracking, safe shot envelopes and full field trials remain required before competition use.



Run `powershell -File tools/auton/check-models.ps1` with a JDK (`JAVA_HOME`) for standalone feeder polarity, cancellation, invalid-input and flywheel recovery checks. These do not validate physical model accuracy.

The current finite-extraction and no-flower simulator results are recorded in the simulator counterpart documentation, `docs/FLOWER-AND-FLOOR-AUTO.md`. The tuned constants are user supplied; neither a successful Android build nor simulator results verifies the real camera/mechanism calibration.

Latest final simulator trials with finite flower pickup: both Rushes scored 15/15, completed two tips, collected all twelve own preplaced balls and parked. Both no-flower routines scored 8/8, completed one tip, collected all four garden balls with zero flower pickups, and parked. Higher recovery scores up to eleven entries occurred in earlier trials but are variable; two no-flower tips are not validated.
