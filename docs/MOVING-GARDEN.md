# Moving shots to the garden and opposite hive

Select **Red Moving Shots + Garden + Hive** or **Blue Moving Shots + Garden + Hive** in **26123 Flowerless**. These new programs drive toward the garden during preload shooting. The earlier Shoot 4 + Garden and Park programs still collect pollen for TeleOp.

## Sequence and setup

Start with four pollen at the Basket 1 start: Red field-centered (-0.360, -1.564259) meters, heading 90 degrees; Blue (+0.360, +1.564259), heading 270 degrees. Use the team's existing drive/Pinpoint configuration. These programs additionally require the configured **limelight**, AprilTag shooting pipeline, and calibrated camera mounting geometry. Current mount assumptions are 10-inch lens height, 7-inch forward offset from the pose origin, 43-degree upward pitch, and an 86-degree shooter elevation.

1. Clear the starting wall before rotating the chassis.
2. **Drive diagonally toward the garden while firing four preloads at the near hive.** The Red destination is (-1.13, -1.05) m, mirrored for Blue. The reference advances immediately at up to 0.15 m/s with 0.30 m/s² acceleration; aim acquisition and flywheel recovery do not stop translation. This deliberately slow diagonal stays inside the existing ballistic model's reachable region. A fast sideways departure did not.
3. Once all four releases are acknowledged and the approach is reached, face the intake toward the garden: Red (-1.15, -1.48) m at 255 degrees; Blue is rotated 180 degrees.
4. Strafe along the garden row to Red (-1.50, -1.48) m at 0.25 m/s, holding the same intake heading, then capture for 0.50 seconds continuously within 1 inch and 3 degrees. Roller and transfer run with the windmill stopped. This is a timed attempt to collect the four available garden balls, not a sensed four-ball inventory.
5. Retreat inboard without turning at the wall, then **shoot toward the opposite hive while crossing** at up to 0.40 m/s and 0.60 m/s². The Red destination is (-1.13, +0.55) m; Blue mirrors it. This leaves the farther teammate parking slot clear.
6. Park in the near alliance slot at Red (-1.55, +0.64) m / Blue (+1.55, -0.64) m.

The robot tries to feed during movement. If a shot is unsafe in transit, it can finish that batch at the destination before the stage timeout. It does not remain at the opening shooting mark to discharge the preloads. Four acknowledged releases are required before garden collection; an empty feeder pulse cannot substitute for an opening ball. Nothing here parks early for a missed position check. Past a stage's time limit the robot keeps correcting onto its mark, the arrival window widens (shots 2.4 in to 6 in; garden capture 1 in/3 degrees to 3 in/8 degrees), and it keeps trying the shots. A garden pass that still cannot capture 2 seconds after its limit counts as made so the final shots are not lost. If the measured start would cross the frame keep-out, the pass starts from its designed origin and the hold drags the robot onto the lane; with no safe lane the pass is skipped. Only the park-reserve check and the 29.7-second stop end a stage early. The opposite hive must be raised and visible before its feeder can open.

## Calculated turn rate

The existing `MovingShotSolver` predicts release position, inherited chassis velocity, yaw-induced muzzle velocity, feed delay, ball flight, drag and spin. The controller solves the ballistic aim now and 0.05 seconds later:

`turnRate = wrap(futureAim - currentAim) / 0.05` radians/second.

It converts the predicted release heading back to a current heading target using the feeder delay, adds a 0.10-second follower lead, and extrapolates between solves. Pedro controls the chassis toward this heading while following the translating reference. Telemetry shows **Calculated aim turn (deg/s)**, measured and predicted travel speed, trajectory validity, feed permission, and acknowledged moving feeds.

Pinpoint supplies a continuous aiming reference, including before tags enter view. Planning uses the desired turn rate instead of feeding measured corrective yaw back into the next heading target. Image-derived range changes validate release safety without abruptly steering the chassis. A feed additionally requires a fresh, consistent Limelight shooting pose, raised-cell confirmation sustained for 0.65 seconds, a predicted entry through the hive opening at the forecast release RPM using measured motion, a trackable turn rate, and chassis clearance. These checks run during translation, including the slow preload approach. Target loss closes the feeder while the transport continues. Missing inventory, a failed shot, or missing tags can prevent a full eight-shot cycle.

The parking time reserve, localization/frame faults and 29.7-second hard stop remain active. Shooter, Intake, TeleOp and Pedro tuning files are unchanged; these new programs use the existing predictive `CycleMechanisms` velocity/feeding helper. Flywheel dips estimate releases, not confirmed hive scores.

## Validation

`tools/auton/check-moving-garden.ps1` compiles against Pedro 3.0.1. It checks heading wrap, delay compensation, rate limits, mirrored sequences, sampled chassis/wall/pot/frame clearance, reserved teammate parking space, a continuous capture dwell, preload ballistic entry while translating and rotating, and a sustained opposite-hive moving-shot window. Feeder/flywheel and vision contract checks run through `tools/auton/check-models.ps1`.

The Android Java compilation and these model/geometry checks pass. Actual OpMode/motor/ball trials in BIOBUZZ-sim exposed and corrected unstable heading feedback. In six subsequent rendered-camera trials, all four preloads fired during movement, all four garden balls were collected, and the robot parked on both alliances. Five trials scored 8/8 entries; one Blue trial scored 7/8 because the last preload arrived during hive tipping. **All garden-batch shots occurred after arrival, not during the crossing.** See the [full simulator results and reproducible test](MOVING-GARDEN-SIM.md). Moving shots toward the garden work in these trials; reliable eight-shot scoring and moving garden-batch shots remain unfinished.

This has not been tested on the physical robot. Validate camera visibility through the turn, moving release RPM/delay, chassis tracking and garden pickup before match use. Start with one preload at the slow approach speed, then increase inventory after checking telemetry and shot placement.

```powershell
# JAVA_HOME must point to a JDK supporting the installed Pedro JAR.
./tools/auton/check-moving-garden.ps1
./tools/auton/check-models.ps1
./gradlew.bat :TeamCode:compileDebugJavaWithJavac --offline --no-daemon
```
