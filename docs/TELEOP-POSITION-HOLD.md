# TeleOp position hold

All four standard `TeleOp_BaseCode*` modes now hold position and heading when all three drive axes are within 0.06 of center for 150 ms. The target is captured once after that delay and remains fixed when pushed, so the robot drives back against another robot's push. Moving either drive stick releases hold in the same loop; the next stop captures a new target. Small bumps are corrected in field coordinates at any heading.

## Real robot tuning

Hold creates its backend with `org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap)`. It uses the existing `localizerConfig`, `drivetrainConfig`, and `foresightConfig` without changing their values or copying gains. Translation uses your forward/strafe controllers; heading uses your heading feedback. Pedro's existing hold-point scaling is honored. This is Pedro Foresight's built-in closed-loop hold, using your current proportional feedback and braking tuning; no separate PID gains or integral term have been invented.

Continue tuning the real robot in `pedro/Constants.java`. This feature has a separate 90% maximum wheel-power limit, independent of the manual drive speed preset. Deadband, engagement delay and power limit are behavior limits in `TeleOpPositionHold`, not replacement robot tuning.

## Controls and fault handling

- Driver 1 left trigger above 0.3 suspends hold for AprilTag alignment. In the single-controller modes this also preserves the existing intake/trigger behavior.
- Hold Driver 1's right stick button to bypass position hold without changing other controls. Releasing it begins a new 150 ms arming interval.
- Driver Station telemetry shows Arming, Holding, Driving, Bypassed, or a fault. Odometry readiness and finite position/velocity are checked before calculating or applying any hold output.
- Pushes from another robot are resisted continuously: the target never moves while the sticks are idle, and telemetry shows "Resisting push" while it is being corrected. The hold only stops if the robot is carried more than 36 inches / 120 degrees from its target (driving back that far could ram something), if odometry jumps faster than any push could move the robot (over 120 in/s or 720 deg/s between updates), on odometry/read/controller errors, or after a gap over 250 ms between idle updates. Move a drive stick (or use bypass) to rearm.
- If Pinpoint initialization fails, manual driving remains available and telemetry reports that hold is unavailable. Stop directly zeros drive motors, bypassing alignment offsets and Pedro's cache.
- Diagnostics, alignment calibration modes, distance-movement tests and autonomous modes do not opt into this feature.

## Validation and robot check

Compiled the changed subsystem and all four standard TeleOps against FTC SDK 12.0.0 and Pedro 3.0.1. `tools/drive/PositionHoldChecks.java` runs 238 checks using the actual Foresight implementation and the repository's real `Constants`: forward/sideways/heading push correction, sustained 30-inch shoves and spins, carry-past-range and odometry-jump faults, rotated field transforms, angle wrap, fixed targets, driver override, fresh targets, bypass, sensor faults, timing interruption and power bounds. The transition tests use a fake pose backend; they do not test physical Pinpoint wiring or motor hardware. Run the helper script with a JDK and a classpath containing the SDK/Pedro/Android dependencies.

On the robot, initialize while stationary and confirm Pinpoint is ready. In an open area, drive, release the sticks, and check that Holding appears. Gently push forward, sideways and rotate; verify it corrects toward the captured pose, including after turning 90 degrees. Check immediate stick override, trigger alignment, right-stick-button bypass and Stop. Available traction, battery and pushing force limit how well it can resist another robot; robot testing is still required.
