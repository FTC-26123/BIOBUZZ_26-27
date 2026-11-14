# Trained Flowers Ramp15

Select `Red Trained Flowers Ramp15 v2` or `Blue Trained Flowers Ramp15 v2`. Telemetry identifies this revision as `RAMP15_V5_FAST_TIMED_CONTACT`. The menu names stay compatible. These are fixed Pedro routes, not live neural inference.

## Setup

| Alliance | Start, Pedro visualizer inches | Heading | Park, Pedro visualizer inches |
| --- | --- | --- | --- |
| Red | (57.143803, 9.180748) | 90 degrees | (11.694882, 95.946850) |
| Blue | (84.356197, 132.319252) | 270 degrees | (129.805118, 45.553150) |

Use the team's Pinpoint origin. Load four pollen and supply four in each own-side flower. Required hardware includes `flowerArm` (CR servo), `touchUp`, `touchDown`, `pinpoint`, and the existing intake/shooter/transfer motors. Missing hardware is reported, not silently skipped.

## Route and shooting

Lower the ramp, shoot four, visit the side-wall flower, collect, retreat, shoot four at Basket 2, visit the end-wall flower, collect, retreat, shoot four again, and park. Blue rotates the route 180 degrees. Both flower lineups remain three inches toward the robot's right while facing the flower.

Each shooting mark is one inch farther from its basket than V4, backward along the existing aim heading. Incoming endpoint/control and outgoing start/control move together. This does not shift the robot's initial placement or parking slot. Shooter speed remains **1450 ticks/second**, trim is cleared, recovery wait remains **0.25 seconds**, and arm direction remains **1**.

## Fast timed flower entry

`FLOWER_PUSH_POWER=0.65` and `FLOWER_PUSH_SECONDS=0.9`. After alignment the drivetrain receives robot-forward power, with bounded heading correction (gain 1.5, maximum turn 0.25) to resist the twist caused by off-center flower contact. Power validation permits up to 0.8 and duration up to 1.5 seconds. This is manual timed entry, not a Pedro translation endpoint or a distance-controlled approach.

Normal completion is **only the 0.9-second timer**. The old forward-coordinate limit, 15-inch travel cutoff, and 3-degree yaw completion have been removed. The motors stop, the intake collects for two seconds, and a straight Pedro retreat starts from measured pose. There is no requirement to hit the former flower endpoint before retreating.

Before entry, admission reserves push + collection + 1.25 seconds for the future retreat + the current parking estimate. If insufficient time remains it parks before starting the impact. Once admitted, the soft parking-reserve check cannot cancel PUSH/COLLECT halfway through. Hardware/geometry faults and the 29.7-second hard deadline still stop motion.

## Tolerances and legitimate stops

| Condition | Behavior |
| --- | --- |
| Ramp lowering / Pinpoint calibration | Stay stopped, update arm and odometry, require down + READY continuously for 0.15 s. |
| Readiness timeout | Stop after max(6 s, configured arm travel timeout + 3 s), measured from the current lower/reseat request. |
| Ramp down switch opens before a flower | One bounded down/reseat attempt at that staging point, then resume the same leg; repeated failure stops visibly. |
| Normal travel / retreat | Position below 3 in, heading below 3 degrees, speed below 4.5 in/s. |
| Flower staging | Position below 0.75 in, heading below 2 degrees, speed below 2.5 in/s. |
| Shooting feed permission | Position below 1 in, heading below 3 degrees, speed below 4.5 in/s, aim settled for 0.25 s. |
| Curve handoff | Within 3.1 in of the next nominal start, or 1.1 in at a flower entry; retreats join from measured pose. |
| Travel timeout | Six seconds, then planned parking. |
| Parking waypoint | Position below 3 in, heading below 3 degrees, speed below 3 in/s; six-second timeout reports failure. |
| Shooting completion / timeout | Four estimated releases or two accumulated FEED seconds; eight-second batch timeout parks. Counts remain estimates. |
| Invalid push parameters, missing hardware, arm fault, lost localization, nonfinite pose/velocity | Stop and report reason. |
| Actual chassis / frame / unrelated wall or flower violation | Stop and report geometry fault. |
| Autonomous deadline | Stop all motors at 29.7 s. |

The selected flower corridor is bounded to canonical forward 40.3–58.5 in, lateral error within 5 in of its offset lineup, and heading within 30 degrees. That is a contact/recovery envelope, not an arrival tolerance. A separate contacted-wall check uses a conservative **10-inch chassis nose and 18-inch width**, derived from the simulator's 9.987-inch nose and 15.771-inch width. The full 15-inch ramp envelope still protects the hive/frame and other walls. Broad corridor angles do not waive the physical chassis-wall check.

The starting pose originally had only 0.016 inches of modeled rear-wall clearance. Departure-only rear contact allowance remains bounded to 0.75 inches in the initial corridor; no global wall checks are disabled. A valid pose inside the parking planner's larger circular margin receives a straight outboard escape before planning, and emergency retreat replans from measured pose.

`Auto result`, `Parking reason`, `Abort details`, `Target errors`, ramp switch readings and `Pinpoint status` distinguish waits from faults. INIT does not move the arm. Every active loop services its real limit-switch controller; STOP cuts arm and drivetrain power.

## Verification

Run `python tools/auton/check-trained-arrival.py` with a JDK on JAVA_HOME/PATH. Set `PEDRO_CORE_JAR` to the installed Pedro core 3.0.1 JAR for the actual-curve geometry checks. Current coverage: **290 route/startup assertions**, **13 real-shooter cadence assertions**, and **82,092 actual-Pedro geometry samples**. Android `:TeamCode:assembleDebug` passes.

Independent agents reviewed stop/cancel conditions, geometry/tolerances, and live simulation separately. The exact requested simulator is `d-mankotia/BIOBUZZ-sim`, commit `0d6d1c5bf563799652e7aeb30ab9a9092761d233`. Live tests load the actual team OpModes, Pedro follower, Shooter, Intake, FlowerArm and Constants. A documented hardware adapter supplies the current transfer motor and CR arm/limit switches; it does not inject balls, teleport poses or change collision/pickup physics. Saved on-robot Robot Settings are not available, so the fixture uses source defaults.

Both nominal live runs at 65%/0.9 s finish at about 22.7 s, with 12 launches, 11 simulated hits, eight flower balls collected and two hive tips. The earlier fast-push run reproduced the reported first-pot stop at 15-degree contact yaw; the corrected steering/contact checks remove that cancellation. Twelve live runs, including exact-final-source smoke tests and scenarios covering both alliances at 55%, 65%, and 75% push power, reduced grip, and a 10.5 V physical supply all finish with the same pickup/tip/launch counts; parking takes about 22.65–24.18 s. The simulator SDK voltage sensor reports 14 V even when physical supply is lower, so these runs do not verify realistic voltage compensation. See the live harness and independent stop/geometry reviews for complete results and remaining coverage gaps.

The simulator models chassis/flower collisions and pickup, but not the exact physical long ramp extension. These are closed-loop simulator results, not proof of real-field impact, tracking or mechanism calibration.
