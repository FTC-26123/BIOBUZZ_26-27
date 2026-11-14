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

Pedro lines up at a staging point about 12 inches before each flower. Both flower lineups are shifted 3 inches toward the robot's right while facing that flower, mirrored for blue. `FLOWER_LINEUP_RIGHT_INCHES=3.0` adjusts staging, push references, retreat endpoints and adjoining curve controls together. The contact corridor follows the offset; flower locations, shooting marks, starting placement and parking coordinates stay fixed. The final approach is a timed robot-forward motor command, not a Pedro endpoint. The nominal reference point still places the modeled ramp tip a quarter inch from the wall. The drivetrain wrapper preserves Pedro's motor cache. Pose/clearance checks, timeouts and parking reserve can stop or shorten the routine; inspect `Parking reason` and `Abort details` if it parks early.

### Timed flower push

Telemetry identifies the updated build as `RAMP15_V4_DEEP_PUSH_CONTINUE`; the red and blue menu names stay the same. Normal travel, flower retreat and fallback travel accept position error below 3 inches. Normal arrival still needs heading error below 3 degrees and speed below 4.5 inches/second; fallback uses a 3-inch/3-degree/3-inches-per-second arrival check and a six-second waypoint timeout. The next normal path permits a 3.1-inch starting offset and joins from measured pose. Path and pose clearance checks still apply to every join.

Flower staging keeps position error below 0.75 inches, heading error below 2 degrees and speed below 2.5 inches/second. Retreat does not demand that pickup-level precision. The shooting feed gate still requires position error below 1 inch, heading error below 3 degrees, speed below 4.5 inches/second and settled aim; entering the shooting state from 3 inches away starts position hold, not immediate feeding. Once lined up at a flower, both alliances command positive robot-forward power, with the intake running and the windmill stopped. There is no quarter-inch endpoint test or position hold during the push.

The current settings are `FLOWER_PUSH_POWER=0.30`, `FLOWER_PUSH_SECONDS=2.5` and `FLOWER_EXTRA_PUSH_INCHES=2.0`. The forward cutoff and maximum travel each allow two inches more than the previous version: canonical forward 57.835 inches, or 15 inches of measured travel from staging. Power must be in (0, 0.45] and duration in (0, 3] seconds. These are bounded contact settings, not a guarantee of distance. The first time/depth/travel limit stops the motors, collects for two seconds and starts a straight retreat from measured pose. A contact-induced heading error above 3 degrees also stops the push and proceeds through collection and retreat, rather than ending the entire auton.

Intentional selected-flower contact/recovery is confined to canonical forward coordinates 40.3 through 58.585 inches, sideways error up to 1.75 inches relative to the shifted lineup and heading error up to 6 degrees. This includes stopping margin beyond the commanded cutoff. The selected wall alone permits four inches of modeled ramp-tip overlap: the old tip geometry plus the requested deeper contact and recovery margin. The physical ramp dimensions still need verification. Other walls, the hive frame and other flowers retain their checks. Pose readiness, finite coordinates, parking reserve and the autonomous deadline remain active.

`Target errors` shows position error, signed heading error and translation speed. `Abort details` saves those values before a parking target replaces the pickup target. If the robot still stops short, record these fields, `Curve` and `Parking reason` to distinguish alignment, tracking and time-budget problems. This update does not change the assumed 15-inch ramp-tip offset or prove physical flower contact.

## Current robot integration

The trained auton initializes a required `FlowerArm` instance using the current continuous-rotation servo `flowerArm` and limit switches `touchUp` and `touchDown`. INIT does not move the arm. At START it commands `down()` and services `update()` each loop; the chassis waits until `touchDown` confirms the ramp is down before beginning the route. Arm power and travel timeout still come from Robot Settings. The code default arm direction is 1, and the trained auton explicitly applies direction 1 at START after saved settings load. This required instance can run when the optional global arm enable is off, without changing saved settings or other OpModes. Both switches and the servo must exist. Missing hardware, contradictory switches or an arm fault stop this auton visibly; each flower push also requires the down switch. STOP turns off the arm servo. Telemetry includes arm status and both switch readings.

Startup also updates odometry while lowering the ramp and waits for both touchDown and Pinpoint READY continuously for 0.15 seconds. It reports a startup timeout after six seconds if readiness never arrives. The original starting geometry had only 0.016 inches of rear-wall clearance; a departure-only corridor now permits up to 0.75 inches of modeled rear contact during leg 1, keeping other walls and the hive frame unchanged. The parking planner uses a larger circular margin than the live rectangle check, so a valid retreat pose inside that margin first gets a straight outboard escape. Emergency retreat replans from measured pose after the staging handoff.

Explicitly selects a shooter target of 1450 ticks/second at START and clears trim before spin-up, using the native shot estimate and feed-permit controls. The trained auton requests a 0.25-second continuous speed-recovery wait after each detected shot (up from the default 0.1 seconds), without changing saved settings or other shooter instances. A detected shot resets that wait even if its speed drop stays inside the allowed window. Its latest PIDF tuning is preserved. Intake uses `intake`, `windmill`, and the current **motor** named `transfer`. Shooter.update() services Intake.update() every travel, pickup and shooting loop so transfer ramping and jam protection remain active. Feeding is requested only in the three shooting states, after settled aim. Pickup runs the intake without feeding the flywheel. All mechanisms stop for parking, stop, and faults.

A batch ends at four estimated flywheel-load events or two accumulated seconds of permitted FEED state. This timed fallback is not confirmation that four balls were released. An eight-second batch timeout preserves a parking opportunity. Flower pickup is timed; neither inventory nor completed hive tips are sensed.

## Validation

Android `:TeamCode:assembleDebug` passes against current main's Shooter, Intake and Constants. The arrival update changes the shared trained auton used by both alliances; no TeleOp, subsystem or Constants tuning is changed.

With a JDK on `JAVA_HOME` or `PATH`, run `python tools/auton/check-trained-arrival.py`. It runs 246 route/startup assertions plus 13 real-shooter cadence assertions across both alliances. Regressions cover delayed/never-ready odometry, startup pose noise, deeper contact, contact turns, collection-to-retreat continuation, three-inch retreat error, the parking-planner margin, emergency retreat and waypoint timeout. Full scripted routes complete both flower visits, all three shot batches and park. Shooting settings remain 1450 ticks/second and 0.25-second recovery. These checks exercise state transitions, not drivetrain physics or actual ball pickup.

Earlier desktop trials of these route coordinates with the previous subsystem revision achieved two simulated tips on each alliance. Those trials do not validate the current motor-transfer mechanism or current Shooter tuning. Real-field pickup, contact, feeder timing and shooting accuracy still require calibration. This routine attempts two tips; it cannot guarantee or measure them.

Optionally set `PEDRO_CORE_JAR` to the installed Pedro core 3.0.1 JAR before running the Python check. `ShiftedRouteChecks.java` samples actual Pedro curves, deeper push/retreat segments, contact recovery extremes, perturbed startup departures and measured-pose parking escapes. All 82,076 samples pass with the 3-inch right offset. This verifies modeled geometry, not real contact dynamics or completed flower tips.
