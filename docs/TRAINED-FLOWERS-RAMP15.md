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

## Current robot integration

Uses the existing Shooter default of 1500 ticks/second, native shot estimate and feed-permit controls. Its latest PIDF tuning is preserved. Intake uses `intake`, `windmill`, and the current **motor** named `transfer`. Shooter.update() services Intake.update() every travel, pickup and shooting loop so transfer ramping and jam protection remain active. Feeding is requested only in the three shooting states, after settled aim. Pickup runs the intake without feeding the flywheel. All mechanisms stop for parking, stop, and faults.

A batch ends at four estimated flywheel-load events or two accumulated seconds of permitted FEED state. This timed fallback is not confirmation that four balls were released. An eight-second batch timeout preserves a parking opportunity. Flower pickup is timed; neither inventory nor completed hive tips are sensed.

## Validation

Android `:TeamCode:assembleDebug` passes against current main's Shooter, Intake and Constants. No existing TeleOp, subsystem, constants or autonomous files are modified by this addition.

Earlier desktop trials of these route coordinates with the previous subsystem revision achieved two simulated tips on each alliance. Those trials do not validate the current motor-transfer mechanism or current Shooter tuning. Real-field pickup, contact, feeder timing and shooting accuracy still require calibration. This routine attempts two tips; it cannot guarantee or measure them.
