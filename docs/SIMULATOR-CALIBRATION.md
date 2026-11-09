# Simulator calibration (BIOBUZZ community simulator)

`tools/auton/sim-calibration.json` is applied on top of the simulator's generic robot by `live-auton-check.cjs`
(and can be entered in the simulator's Robot setup). Values and why:

| Setting | Value | Basis |
|---|---|---|
| `shooterMaxTicks` | 2800 | HD Hex 6000 RPM x 28 ticks/rev. The team `Shooter` kF (0.00035) implies about 2857. The default 2240 could not reach the 1500 ticks/s the code needs, so it never fed. |
| `driveRpm` | 440 | Reproduces the forward speed measured for Pedro (`maxAchievableForwardVelocity` 74.58 in/s = 1.894 m/s): the simulator peaks at 1.90 m/s. |
| `angleDeg` | 85 | The robot shoots at about 86 degrees; the simulator's limit is 85. |
| `speedFactor` | 0.56 | Not measured. Chosen as the centre of the 0.53-0.59 band where a shot from the known-good Basket 1 mark (1500 ticks/s) enters the hive. Measure real exit speed to replace this. |

Not calibrated: strafe speed (simulator ~1.7 m/s peak vs Pedro 1.54), deceleration, flywheel droop/recovery, battery sag.
Shot hits are therefore a plausibility check, not proof of accuracy on the real field.

## Simulator fixes needed for this robot (in the community simulator folder)

- `BioBuzzBot.place`: wall clamp used the half-diagonal and pushed an 18 in robot 9.4 cm off its start mark; it now uses the rotated footprint.
- `BioBuzzBot`: intake power ignored motor direction (the team `Intake` is REVERSE with -0.8 = in), so balls were ejected instead of launched.
- `BioBuzzBot`: added the `transferServo` the team `Intake` needs (open gate when its power is below -0.1), and launches only release while the windmill is feeding, as the `Shooter` pauses feeding by stopping it.
- `StudioServer`: port can be set with `-Dbiobuzz.port=` (host check follows it).

The simulator places the robot at the program's Pedro/Pinpoint start pose during INIT (no manual step; `live-auton-check.cjs` verifies it and only falls back to placing by hand on older simulators). Simulator heading 0 faces +y, which is Pedro heading 90 degrees.
Red start: x -0.36, y -1.564259, heading 0. Blue start: x +0.36, y +1.564259, heading pi (180 degrees in the UI).
Basket 2 Wait 15 starts at x -/+0.245, y +/-1.564259 on the opposite wall (Red faces -y, so heading pi).

## Mechanics added to the simulator

- **Contact intake**: the rollers act only on a ball touching them across the mouth (they reach 4.5 cm past the frame, setting `intakeReach`), so the robot must drive onto the ball. A touching ball rides the rollers in and is taken after the capture time (0.075 s). Rear, sides, a full magazine, an idle intake and filtered balls (**Intake takes**: everything / pollen only / NECTAR only) are only pushed. With this model the Garden sweep reaches 2 of the 4 wall balls; the two nearest the corner are outside the mouth at the sweep's end point (the earlier 22 cm suction model pulled them in sideways).
- **Shot variation**: each launch varies exit speed (2.5 %, one standard deviation), launch angle (0.3 degrees) and aim (1.0 degree), seeded so runs repeat. The landing prediction and shot log show the nominal shot, so the scatter is visible.
- **Shot tracking**: every launched ball is a `Physics.Shot` with its arc, apex, flight time and a verdict: `HIVE` (with where it crossed the mouth), `FLOWER`, or `MISS` with a reason (below the mouth, never reached the plane, hit the rim or frame, HIVE already tipped, left the field).
- **Arc display and log**: the last four arcs are drawn on the field (green HIVE, red miss, yellow in flight) and the **Shot log** panel lists the last twelve shots.
- **Motors & servos panel** (Drive page): each device's configured direction, command, physical spin (+ clockwise, - counter-clockwise) and meaning (intake IN/OUT, windmill FEEDING, gate OPEN/CLOSED), plus a CHASSIS line with the robot's actual motion (forward/back, strafe, turn) to catch reversed drive motors. The same data is in the `/api/state` fields `motors`, `shotLog` and `arcs`.

## HIVE tip timing and feeder transport

- **Tip time**: the official manual (10.5.1, TU05) defines a tip (bi-stable HIVE reaches the other stable state, then the damper touches the frame) but gives no duration. The only measurement found is team 14343's Chief Delphi build thread: **4.76 s** from first pollen contact to a scored tip with 8 pollen (one trial). Their second trial, 2 nectar + 5 pollen, took 2.78 s but they doubted their hive calibration. The simulator uses 4.76 s at the threshold load and shortens it as load rises (time scales with (8 / load)^2, an assumption).
- **Mid-swing balls**: reaching the load only starts the swing. The loaded cell accepts balls while its mouth is open (the opening shrinks to nothing at half the swing), so a ball that gets in mid-swing adds load and the HIVE tips sooner. Later balls find the mouth turned away. The manual warns launching during a tip can disrupt it; that is not modelled.
- **Feeder transport**: a ball needs about 0.14 s to travel the feeder (0.6286 m CAD path) before the flywheel releases it. Without that, the team `PreloadShotCounter` (which needs about 218 encoder ticks of feeder travel per release) never counted anything and no auton could stop on "four releases".
- **Result**: Red Basket 1 now counts releases 1, 2, 3, 4, switches the intake off the moment the fourth ball leaves, and parks at about 15 s with exactly four shots.
