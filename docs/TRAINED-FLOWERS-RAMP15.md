# Trained Flowers Ramp15 v2

Select `Red Trained Flowers Ramp15 v2` or `Blue Trained Flowers Ramp15 v2`.
These are fixed Pedro routes exported from the trained flower policy, not live neural inference.

## Setup

| Alliance | Start, Pedro visualizer inches | Heading | Park, Pedro visualizer inches |
| --- | --- | --- | --- |
| Red | (57.143803, 9.180748) | 90 degrees | (11.694882, 95.946850) |
| Blue | (84.356197, 132.319252) | 270 degrees | (129.805118, 45.553150) |

Use the existing Pinpoint pose origin and tuned Pedro Constants. The ramp tip extends 15 inches forward of that origin; modeled width is 18 inches and rear extent is nine inches. Load four pollen and provide four pollen in each own-side flower. No teammate or camera is required.

## Routine

Shoot four at Basket 1, approach the side-wall flower straight, collect for two seconds, back out, shoot at Basket 2, approach the end-wall flower straight, collect for two seconds, back out, shoot at Basket 2 again, and park. Both shooting marks are two inches closer along the existing aim direction. Blue is a 180-degree mirror. The route stays on the alliance half and goes around the hive frame.

Pedro lines up at a staging point about 12 inches before each flower. The final approach is a timed robot-forward motor command, not a Pedro endpoint. The nominal reference point still places the modeled ramp tip a quarter inch from the wall. The drivetrain wrapper preserves Pedro's motor cache. Pose/clearance checks, timeouts and parking reserve can stop or shorten the routine; inspect `Parking reason` and `Abort details` if it parks early.

### Timed flower push

Telemetry identifies the updated build as `RAMP15_V3_TIMED_PUSH_3IN_ARM`; the red and blue menu names stay the same. Normal travel accepts position error below 3 inches, heading error below 3 degrees and speed below 4.5 inches/second. Normal fallback travel also accepts a 3-inch position error. The next normal path permits a 3.1-inch starting offset and joins from measured pose, so the old 1.1-inch start check cannot reject a normally accepted endpoint. Path and pose clearance checks still apply to every join.

Flower staging and retreat keep position error below 0.75 inches, heading error below 2 degrees and speed below 2.5 inches/second. The shooting feed gate still requires position error below 1 inch, heading error below 3 degrees, speed below 4.5 inches/second and settled aim; entering the shooting state from 3 inches away starts position hold, not immediate feeding. Once lined up at a flower, both alliances command positive robot-forward power, with the intake running and the windmill stopped. There is no quarter-inch endpoint test or position hold during the push.

The starting settings are `FLOWER_PUSH_POWER=0.30` and `FLOWER_PUSH_SECONDS=1.25`. These are initial field-tuning values, not a calculated guarantee of contact or travel distance. Power must be in (0, 0.45] and duration in (0, 2] seconds. The push ends at the time limit, or sooner at a maximum measured travel of 13 inches or half an inch beyond the old reference point. These distance limits are overtravel guards; they do not demand accurate arrival. It then cancels manual drive, stops the motors, collects for two seconds and uses a straight Pedro retreat from the measured pose. It does not require the timed push to finish at the old path endpoint.

Intentional selected-flower contact is permitted only during push, collection and retreat within a bounded corridor: canonical forward coordinate 42.3 to 56.085 inches, sideways error up to 1.25 inches and heading error up to 3 degrees. In that corridor, the selected flower contact exception is wider and that flower's wall permits up to one inch of modeled ramp-tip overlap. This accounts for intentional contact; it is not permission to ignore other walls or the hive frame. Pose readiness, finite coordinates, other-flower checks, parking reserve and the autonomous deadline remain active. The exact ramp shape, wall contact and resulting ball pickup need a real-field trial.

`Target errors` shows position error, signed heading error and translation speed. `Abort details` saves those values before a parking target replaces the pickup target. If the robot still stops short, record these fields, `Curve` and `Parking reason` to distinguish alignment, tracking and time-budget problems. This update does not change the assumed 15-inch ramp-tip offset or prove physical flower contact.

## Current robot integration

The trained auton initializes a required `FlowerArm` instance using the current continuous-rotation servo `flowerArm` and limit switches `touchUp` and `touchDown`. INIT does not move the arm. At START it commands `down()` and services `update()` each loop; the chassis waits until `touchDown` confirms the ramp is down before beginning the route. Arm power, direction and travel timeout still come from Robot Settings. This required instance can run when the optional global arm enable is off, without changing saved settings or other OpModes. Both switches and the servo must exist. Missing hardware, contradictory switches or an arm fault stop this auton visibly; each flower push also requires the down switch. STOP turns off the arm servo. Telemetry includes arm status and both switch readings.

Uses the existing Shooter default of 1500 ticks/second, native shot estimate and feed-permit controls. Its latest PIDF tuning is preserved. Intake uses `intake`, `windmill`, and the current **motor** named `transfer`. Shooter.update() services Intake.update() every travel, pickup and shooting loop so transfer ramping and jam protection remain active. Feeding is requested only in the three shooting states, after settled aim. Pickup runs the intake without feeding the flywheel. All mechanisms stop for parking, stop, and faults.

A batch ends at four estimated flywheel-load events or two accumulated seconds of permitted FEED state. This timed fallback is not confirmation that four balls were released. An eight-second batch timeout preserves a parking opportunity. Flower pickup is timed; neither inventory nor completed hive tips are sensed.

## Validation

Android `:TeamCode:assembleDebug` passes against current main's Shooter, Intake and Constants. The arrival update changes the shared trained auton used by both alliances; no TeleOp, subsystem or Constants tuning is changed.

With a JDK on `JAVA_HOME` or `PATH`, run `python tools/auton/check-trained-arrival.py`. It compiles the actual shared auton, FlowerArm and frame-clearance code against scripted hardware/Pedro substitutes, then runs 126 checks across both alliances. Startup checks exercise the real arm controller's down command, touchDown stop, missing sensor, contradictory switches, timeout and external stop. Route checks cover three-inch normal travel, both timed pushes, motor stop before collection, measured-pose retreat and remaining interlocks. These checks exercise state transitions, not drivetrain physics or real pickup.

Earlier desktop trials of these route coordinates with the previous subsystem revision achieved two simulated tips on each alliance. Those trials do not validate the current motor-transfer mechanism or current Shooter tuning. Real-field pickup, contact, feeder timing and shooting accuracy still require calibration. This routine attempts two tips; it cannot guarantee or measure them.
