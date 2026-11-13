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

Flower approaches target a quarter-inch ramp-tip gap to the wall. Pedro wheel-demand ceilings are 45%, reducing to 22% within two inches of the contact target. The drivetrain wrapper preserves Pedro's motor cache. Pose/clearance checks, timeouts and parking reserve can stop or shorten the routine; inspect `Parking reason` and `Abort details` if it parks early.

### Arrival fix

Telemetry identifies the updated build as `RAMP15_V2_ARRIVAL_FIX`; the red and blue menu names stay the same. Staging before each pickup accepts position error below 0.75 inches, heading error below 2 degrees and speed below 2.5 inches/second. The final wall-contact stop still requires errors below 0.25 inches and 1 degree, with speed below 1.5 inches/second. The route now keeps following its path until arrival instead of switching to position hold 1.2 inches early. Arrival takes priority over the six-second drive timeout on the same loop; clearance checks and the overall parking reserve still take priority.

`Target errors` shows position error, signed heading error and translation speed. `Abort details` saves those values before a parking target replaces the pickup target. If the robot still stops short, record these fields, `Curve` and `Parking reason` to distinguish alignment, tracking and time-budget problems. This update does not change the assumed 15-inch ramp-tip offset or prove physical flower contact.

## Current robot integration

Uses the existing Shooter default of 1500 ticks/second, native shot estimate and feed-permit controls. Its latest PIDF tuning is preserved. Intake uses `intake`, `windmill`, and the current **motor** named `transfer`. Shooter.update() services Intake.update() every travel, pickup and shooting loop so transfer ramping and jam protection remain active. Feeding is requested only in the three shooting states, after settled aim. Pickup runs the intake without feeding the flywheel. All mechanisms stop for parking, stop, and faults.

A batch ends at four estimated flywheel-load events or two accumulated seconds of permitted FEED state. This timed fallback is not confirmation that four balls were released. An eight-second batch timeout preserves a parking opportunity. Flower pickup is timed; neither inventory nor completed hive tips are sensed.

## Validation

Android `:TeamCode:assembleDebug` passes against current main's Shooter, Intake and Constants. The arrival update changes the shared trained auton used by both alliances; no TeleOp, subsystem or Constants tuning is changed.

With a JDK on `JAVA_HOME` or `PATH`, run `python tools/auton/check-trained-arrival.py`. It compiles the actual shared auton and frame-clearance code against scripted hardware/Pedro substitutes, then runs 40 red/blue staging, approach, precise-contact, timeout and diagnostic assertions. The pre-fix source fails the staging-to-approach regression. These checks exercise state transitions, not drivetrain physics or real pickup.

Earlier desktop trials of these route coordinates with the previous subsystem revision achieved two simulated tips on each alliance. Those trials do not validate the current motor-transfer mechanism or current Shooter tuning. Real-field pickup, contact, feeder timing and shooting accuracy still require calibration. This routine attempts two tips; it cannot guarantee or measure them.
