# Pollen cycle autonomous

## Driver Station programs

- **26123 Red Rush - pollen cycles** (`RedRush`)
- **26123 Blue Rush - pollen cycles** (`BlueRush`), mirrored field route

These are the current two-tip simulator baseline, ported from BIOBUZZ-sim main commit `9c28f9e429bb098fabf1819c1376b188fa3368f6`. They prioritize flower pollen, all twelve alliance-side preplaced pollen balls, and useful shooting transfers toward the next pickup. They can attempt further cycles and a partial teleop pickup while reserving time to park. Nectar is unsupported. Third/fourth tips are not verified. Neither program has been validated on the physical robot.

Only these two wrappers are registered. Older flower/adaptive wrappers failed a fresh two-tip qualification (zero or one completed tip and a far-fire timeout), and nectar-dependent tip wrappers are excluded. Their shared route definitions remain internal; they are not additional Driver Station programs.

## Required setup before field testing

1. Run the existing Pedro drivetrain/localizer and Foresight tuners. Paste the measured `ForesightConfig` produced by `ForesightTuner` into `Constants.foresightConfig`. It deliberately has no guessed default: Init reports the missing tuning rather than returning a null follower. Existing team drivetrain directions and Pinpoint offsets are preserved and still need physical verification.
2. Configure the required hardware names: `frontLeftMotor`, `frontRightMotor`, `backLeftMotor`, `backRightMotor`, `pinpoint`, `shooter`, `intake`, `windmill`, `gate`, `limelight`. Gate closed/open positions are 0.50/0.27. Match the existing subsystem conventions: shooter FORWARD with positive encoder velocity; intake REVERSE with negative intake power; windmill FORWARD with positive power. Verify these with the robot raised before loading balls. Shooter uses RUN_USING_ENCODER; intake and windmill use RUN_WITHOUT_ENCODER with BRAKE.
3. Configure real Limelight pipeline **0** for pollen color detections only and pipeline **1** for hive AprilTags. Red near/far tag groups are 34–37/30–33; blue near/far are 42–45/38–41. Confirm those IDs against the actual field. Fire requires a valid expected-cell tag and total capture/processing age at most 180 ms. A camera frame alone does not guarantee a shot.
4. Measure the camera mount. Ground projection currently assumes height **0.305 m**, pitch **+10 degrees**, zero yaw/roll and zero horizontal offset relative to robot pose. Physically match these assumptions or update the projection; this port does not move/configure a simulated camera.
5. Measure shooting and feeder behavior. Current estimates assume 6000 RPM motors, 28 encoder ticks/rev, a 72 mm roller, 0.6285866147 m feed path, 1:1 reduction, 25% transfer, 35 ms gate delay, and 350 ms minimum spacing. The flywheel forecast, velocity-loop response, load-event threshold, ball drag/backspin, launch slip and gate pulse timing require real tests and tuning. A flywheel load event is a feed estimate, not a scoring sensor. Do not treat simulator gains/shot percentages as hardware calibration.
6. Verify the muzzle geometry: forward 0.059119 m, right approximately zero, height 0.288940 m, launch angle 55 degrees. The chassis envelope is approximately 0.401 by 0.458 m. Path reference limits are 1.0 m/s and 4.0 m/s²; these are requested limits, not measured 435 RPM / 104 mm drivetrain capabilities. Tune actual acceleration, braking and tracking before fast shots.
7. Start with four pollen preloads and an otherwise reset field. Coordinates are field-centered inches, +X right, +Y up; Pedro heading zero points +X. Red starts at (-0.360, -1.564259) m, heading 90 degrees; blue is its 180-degree mirror, (+0.360, +1.564259) m, heading 270 degrees. Init displays the placement. Verify localizer heading and alliance-half clearance on the real field.

## Behavior and limits

The controller estimates release pose from odometry, feeder travel and flywheel recovery. A drag/backspin trajectory solver compensates for robot translation and rotation. Scoring paths move one way toward useful pickups, with a frame-support bypass. Fast-shot permission requires current speed over 0.40 m/s and predicted release speed over 0.75 m/s, a valid trajectory, expected fresh tag, and chassis clearance. If unsafe while moving, the remaining shot can finish at the pickup approach; it does not add artificial shooting laps.

Tipping is not read from simulator truth. A 3.15 s flight/hinge allowance follows relevant feed stages; collection uses projected pollen detections. Parking uses the measured pose and a reserved return budget, with a hard 29.7 s cutoff. A timeout can terminate early or trigger parking; a stopped drivetrain is not proof of parking. Stop closes the gate and stops mechanisms, follower and camera.

The shared controller includes older route branches for compatibility, but this PR enables only Rush. It imports no simulator hardware stubs, field truth or `SimPedro` tuning. It keeps the team's Java 8 build configuration, replacing source records/type inference with ordinary immutable classes and explicit types.

## Validation

A recent matching simulator run on the source revision recorded RedRush 16/16 shots, two completed tips, three fast shots and parking at 28.43 s; BlueRush recorded 17/17, two tips, three fast shots and parking at 28.41 s. Timing/scatter varies between runs. These results describe the simulation before the hardware-direction and follower integration changes; they do not certify the port on a robot.

Build the real Android project with `gradlew.bat :TeamCode:assembleDebug`. Physical calibration, feed confirmation, camera alignment, drivetrain tracking, safe shot envelopes and full field trials remain required before competition use.

Run `powershell -File tools/auton/check-models.ps1` with a JDK (`JAVA_HOME`) for standalone feeder polarity, cancellation, invalid-input and flywheel recovery checks. These do not validate physical model accuracy.
