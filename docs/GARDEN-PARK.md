# Shoot four, collect our garden, and park

Select **Red Shoot 4 + Garden and Park** or **Blue Shoot 4 + Garden and Park**. These are separate new files directly in `Auton`. The working Shoot 4 and Park programs remain available. The new routines shoot only the opening batch, collect only our alliance's garden and park carrying that pollen into TeleOp. They never target the opposite garden or a second basket.

![Red and Blue garden-and-park routes on the official FIRST CAD, showing starts, initial shots, own-garden sweeps, parking and reserved teammate spaces.](garden-park-map.png)

## Setup and sequence

Start with four pollen at the same mark as Shoot 4 and Park. Red starts at field-centered (-0.360, -1.564259) meters, Pedro heading 90 degrees; Blue mirrors by 180 degrees, (+0.360, +1.564259), heading 270 degrees. Initialize Pinpoint at the selected starting mark. No flower mechanism or camera is used.

1. Drive one straight leg from the wall and shoot four preloads at the revised mark: Red visualizer (57.0146, 27.4806), Blue (84.4854, 114.0194), exactly one inch farther from the hive along the aim line. The existing Shooter class retains its default **1500 encoder ticks/second** and its PIDF/feeder readiness logic.
2. Stop the shooter. Drive one straight leg to our own garden, finishing the turn within the first 60% of the leg, with the intake angled 15 degrees toward the corner (Red 255 degrees / Blue 75 degrees). Run the existing Intake subsystem and keep its windmill stopped, preventing the collected pollen from being shot.
3. Sweep toward our garden corner with a moving hold target at 0.25 m/s, require the endpoint within 0.20 inch and 1 degree, then dwell 0.50 seconds for capture. Pickup is timed; there is no inventory sensor. Expect up to four pollen if the garden remains available.
4. Stop all mechanisms. Drive one continuous curve inboard of the fixed flower pot and outside the metal frame into our slot.
5. Park in our near slot, leaving the farther teammate slot clear. Keep the collected pollen for TeleOp; there is no second firing stage.

Park centers are Red visualizer **(9.73, 95.95)** / Blue **(131.77, 45.55)**. Garden-and-Park finishes facing 270 degrees Red / 90 degrees Blue, avoiding a half-turn. Two 18-inch square footprints have the same 3.65-inch gap to the reserved teammate slot. Coordinate actual teammate traffic; the route does not detect moving robots.

## Existing Intake subsystem

The new routines and existing fixed-shot autons use `subsystems.Intake`, with Shooter receiving that same instance. The older `CycleMechanisms` helper also now delegates roller, windmill and transfer commands to this subsystem instead of setting their directions, modes or powers itself. Its direct intake motor reference reads the encoder only.

The branch includes the latest main-branch Intake change, **INTAKE_POWER = 0.80**, without modifying it. Hardware names, directions, inward roller/transfer behavior, windmill commands and stopping behavior come from that class. TeleOp, Shooter, Intake and tuned Pedro constants are unchanged relative to latest main. Park Only does not operate an intake.

## Simulation evidence and limits

The reproducible study uses unmodified simulator `Physics`, `Settings` and `ContactMath`: 86-degree launch elevation, finite capture and feeder travel, ball flight/collisions and damped hive swing. Each case resets the standard field with four robot preloads and the original garden balls. No extra balls or teammate injection are used. The fixture reads the current Intake power constant. Red and Blue produced identical results:

| Bounded drive profile | Parked | Final inventory | Result |
| --- | --- | --- | --- |
| 1.50 m/s, 2.50 m/s² | 12.330 s | Four own-garden pollen | Four opening shots, four entries, one tip |
| 1.30 m/s, 1.60 m/s² | 13.685 s | Four own-garden pollen | Four opening shots, four entries, one tip |
| 0.80 m/s, 0.60 m/s² | 19.090 s | Four own-garden pollen | Four opening shots, four entries, one tip |

All six cases acquired **zero opposite-garden balls, zero flower pollen and zero nectar**, respecting four-ball capacity. These ideal-motion planning cases produce four entries. The actual controller/motor runs below produce three entries from four shots; do not assume planning-model shot accuracy on hardware.

These are **planning-model results, not an end-to-end run of the FTC OpMode, tuned Pedro follower or physical motors**. Translation follows bounded acceleration/deceleration trajectories; the fixture gates its feeder with simulator release counts and an estimated 0.4-second spin-up. Real code uses the existing flywheel-load/intake-encoder release estimator. Validate pickup at the garden edge, actual feeding and tracking on the robot. The tested drive profiles do not retune Pedro constants.

Incomplete opening shots, stage timeout or the parking reserve cause an early return to park. The 29.7-second cutoff stops all motors; localization or frame-clearance faults stop immediately. Telemetry reports estimated releases and a timed pickup, not an invented sensed inventory or tip count.

## Actual autonomous and Pedro follower verification

The previous planning study missed a controller-level failure. In an actual OpMode run, the team's unchanged Shooter released only three preloads before the 10-second firing timeout. The routine parked with one preload still held and never entered the garden stage. The firing allowance is now 14 seconds, while the parking reserve and 29.7-second hard stop remain active. Partial release counts remain visible after a timeout.

The old garden sweep also relied on about 3 mm of modeled intake-edge reach while accepting a 0.65-inch endpoint error. The revised corner-facing approach ends farther inboard at Red (-1.50, -1.48) m, heading 255 degrees, mirrored for Blue. It provides more pickup margin without driving further into the wall.

The repeatable live test runs the production Java OpModes, team Shooter/Intake, unchanged tuned Pedro follower, SDK motor/Pinpoint stand-ins and simulator drivetrain/ball plant. It does not inject poses or balls. The isolated simulator hardware adapter supplies the missing transfer CRServo and maps its inward state to the simulator gate; the simulator checkout is untouched. Results and sampled trajectories are saved in [the validation folder](validation/garden-follower-results.json).

| Actual-code routine | Autonomous elapsed | Shots / entries | Garden pollen held at park |
| --- | --- | --- | --- |
| BlueBasket1Shoot4Park | 16.389 s | 4 / 3 | 0 |
| BlueShoot4GardenPark | 20.531 s | 4 / 3 | 4 |
| RedBasket1Shoot4Park | 15.782 s | 4 / 3 | 0 |
| RedShoot4GardenPark | 20.210 s | 4 / 3 | 4 |

All four completed parking, stopped their motors and acquired no flower pollen or nectar. Garden routines collected only their four own-garden pollen. These remain simulator results: physical feeding, slip and intake dimensions require a robot trial.

### Pedro visualizer geometry

[Red project](pedro/RedGardenPark.pp.json) and [Blue project](pedro/BlueGardenPark.pp.json) contain the exact four route legs (12 stop-and-go segments before the continuous-route redesign), headings, an 18-inch footprint, frame, fixed flower pots and reserved teammate space. Both pass the upstream Pedro Visualizer's own `calculatePathTime` and `findCollisions` engine at 0.20-inch / 1-degree sampling with **zero collision intervals**. A deliberately invalid path into the frame is rejected as a negative control. This checks geometry; it does not replace the actual follower trial.

The browser's detached file input could not import the projects reliably, so verification ran the same upstream geometry engine directly from [Pedro-Pathing/Visualizer commit 97104e0](https://github.com/Pedro-Pathing/Visualizer/tree/97104e027af00d49b8bd61e29bcb74a9d7e62b90). The engine check does not access browser storage or modify the visualizer source. Open the exported projects in the visualizer for interactive playback.

## Reproduce

With the simulator alongside this robot checkout, including its built SDK stand-ins, bundled runtime, Pedro library and official field tessellation:

```powershell
.\tools\auton\run-garden-park-sim.ps1 -SimulatorRoot ..\sim-publish
python tools/auton/render-garden-park-map.py
python tools/auton/export-pedro-projects.py
.\tools\auton\run-live-autons.ps1 -SimulatorRoot ..\sim-publish
node tools/auton/check-pedro-visualizer.mjs build/pedro-visualizer-reference
```

The study compiles the current production route, current Intake and simulator plant, checks the main route plus fallback paths, runs six reset-field cases and exports map coordinates. There are **462,564 route assertions** covering mirrored geometry, shared shooting marks, frame/wall clearance, fixed flower-pot clearance and the reserved teammate footprint; the physics cases each also sample 6,000 rotated footprints. The map uses official FIRST BIOBUZZ STEP v26-27.2 data bundled in the simulator; see its [field provenance](https://github.com/d-mankotia/BIOBUZZ-sim/blob/main/FIELD-SOURCES.md).

The upstream geometry check requires Node 24+ with native TypeScript support and a checkout of the pinned Visualizer source at the supplied path. No package install or browser-state access is used. The live runner writes its isolated SDK adapter and traces under `build/` and stops its own simulator after the four trials.
