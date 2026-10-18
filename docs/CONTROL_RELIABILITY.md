# Control reliability changes

These changes address code behavior. They do not tune Pedro Pathing or replace robot testing.

## Driver behavior

- Hold the shooter right trigger to spin up and request feeding. Releasing it cancels automatic feeding while the flywheel can remain spinning. The right bumper stops the flywheel, intake, and automatic feeding and closes the gate.
- Readiness requires measured flywheel velocity within 50 encoder ticks/second of the selected target for 150 milliseconds. Underspeed, overspeed, invalid velocity, and changing the preset reset this dwell. Feeding pauses when the flywheel leaves that band.
- Manual intake stop, reverse, and custom-power commands take priority over automatic feeding in that loop. Manual gate close also cancels the firing request. Avoid holding the intake and firing controls together; use the firing control to operate automatic feeding.
- X opens the gate as an explicit manual/service action; it does not require flywheel readiness. Y closes it. A missing optional gate reports `Not installed`.
- In the single-controller TeleOp, hold A for vision rotation alignment. The left trigger operates intake, removing the previous collision between intake and alignment. X/Y operate the gate.
- Combined mecanum inputs are normalized before applying the selected speed multiplier. This preserves their relative wheel powers and the selected power ceiling.
- Custom positive intake power now uses the same windmill direction as normal intake; negative power reverses both mechanisms.

Consult each OpMode's telemetry for its controller assignment. Mechanism test OpModes retain their diagnostic purpose.

The AprilTag relay calibration measures gains in drivetrain input units. Its results depend on the selected drive speed multiplier; validate the gains at the same setting before using a different multiplier. This PR does not change the calibration formula or supply new gains.

## Vision and odometry

Vision consumers reject invalid or stale results. Autonomous alignment requires distinct camera frames to progress its alignment dwell; polling one frozen frame cannot establish a lock. The four-tag requirement remains because inferring a cluster center from fewer tags needs verified physical tag offsets.

Odometry updates cache one sensor sample for the loop, and the odometry test connects its movement controller to the drivetrain. Stopping a movement clears injected offsets before commanding zero power.

## Autonomous stage limits and visual arrival

Both autonomous routines now use bounded stages with a 27-second execution budget and a five-second parking reserve. Path stages have an eight-second deadline and stop after two seconds without meaningful pose progress (0.25 inches or one degree). These are configurable execution guards, not Pedro tuning values; verify them against measured route times. A failed aim/search or an active-stage deadline skips later collection/aim attempts and enters a bounded park when the Pinpoint reports READY and its pose is finite. Sensor faults, invalid pose, no-progress faults, and the overall deadline stop the drivetrain directly rather than attempting an untrusted park.

Path completion now waits for Pedro's full `isBusy()` completion criteria instead of only reaching the parametric endpoint. Stage name, elapsed time, result, and failure reason are reported in telemetry. Camera-derived and path commands are selected before the follower's motor update.

The yellow-ball seeker shares one parsed camera observation between movement, completion, and telemetry. It rejects invalid angles/areas, stale results, and wrong pipelines. Close off-axis balls keep turning with forward power zero. Visual arrival requires both the existing closeness threshold and horizontal error within four degrees, held for 150 ms across distinct frames. A frozen frame cannot confirm arrival. This confirms only visual positioning; it is not a possession sensor and does not add autonomous intake/firing.

The existing color-height model and pixel thresholds are preserved setup assumptions and still need camera validation. HiveAim retains target-relative tag geometry and mounting transforms; a robot's field `botpose` is not a target-relative forward/right/up vector.

## Remaining robot-dependent work

`Constants.create()` now builds a real Pedro 3 follower once `Constants.foresightConfig` is supplied. Until then it fails with an explicit setup message. Pedro 3.0.1 requires robot-specific controller, braking, maximum velocity, and deceleration values; values from another robot would not be a valid repair. Paste the complete generated configuration after the team's tuning session. Existing motor directions and odometry geometry still need verification on this robot.

The autonomous routines currently drive and calculate a proposed shot; they do not collect or fire game pieces. Completing autonomous scoring requires measured camera/shooter heights, opening offsets, ball-speed conversion, and a verified way to achieve the requested launch angle. The `NaN` geometry placeholders remain intentional blockers. No autonomous firing or invented physical measurements are introduced.

Before field use, verify:

1. With mechanisms clear, confirm normal/custom intake and reverse directions and gate endpoints.
2. Confirm the stop bumper wins when held together with the firing trigger; intake stop/reverse and gate close must also interrupt feeding.
3. Observe measured velocity during spin-up, repeated shots, preset changes, and recovery. Confirm feeding waits for stable speed and pauses on droop. Adjust the readiness band/dwell from measured shot behavior if needed.
4. Confirm combined driving inputs respect each selected speed setting.
5. Block/disconnect the camera and verify alignment releases control safely. Verify fresh frames from the intended pipeline before autonomous testing.
6. Test odometry motion and stop at low power with space around the robot before trusting autonomous paths.

Run `python tools/test_controls.py` with a JDK available through `JAVA_HOME` or `PATH` for the focused software regression checks. These exercise production control behavior with hardware fakes. A successful build or regression run does not establish motor direction, shot accuracy, camera geometry, or autonomous path performance.

Run `python tools/test_autonomous.py` for autonomous acquisition, snapshot reuse, failure routing, cancellation, localization guards, and deadline checks. Both complete autonomous OpModes are compiled in that harness and the real Android build. GitHub Actions runs both suites.

## API references

- [Pedro follower construction](https://pedropathing.com/docs/pathing/tuning/constants)
- [Pedro 3.0.1 required Foresight configuration fields](https://github.com/Pedro-Pathing/PedroPathing/blob/v3.0.1/core/src/main/java/com/pedropathing/algorithm/ForesightConfig.java)
- [FTC LLResult freshness and source timestamp API](https://javadoc.io/static/org.firstinspires.ftc/Hardware/11.2.0/com/qualcomm/hardware/limelightvision/LLResult.html)
