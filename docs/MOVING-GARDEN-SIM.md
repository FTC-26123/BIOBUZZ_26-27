# Actual moving-garden autonomous simulator trials

On November 8, 2026, the actual Red/Blue `GardenMovingShots` OpModes were run in [BIOBUZZ-sim](https://github.com/d-mankotia/BIOBUZZ-sim), including Pedro feedback, team `Constants`, SDK motor/Pinpoint stand-ins, flywheel/feeder dynamics, rendered AprilTag camera frames, finite garden pickup and ball/hive physics.

**The moving preload approach and garden pickup worked in all six corrected trials. Reliable 8/8 scoring and moving shots from the garden batch are not established.**

| Trial | Alliance | Actual shots | Hive entries | Moving preloads | Garden collected | Garden-batch moving shots | Parked at |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | Red | 8 | 8 | 4 | 4 | 0 | 19.622 s |
| 1 | Blue | 8 | 7 | 4 | 4 | 0 | 19.929 s |
| 2 | Red | 8 | 8 | 4 | 4 | 0 | 19.759 s |
| 2 | Blue | 8 | 8 | 4 | 4 | 0 | 18.732 s |
| 3 | Red | 8 | 8 | 4 | 4 | 0 | 19.422 s |
| 3 | Blue | 8 | 8 | 4 | 4 | 0 | 19.416 s |

All six collected zero flower pollen and zero nectar, finished with no held balls, and stopped drive/intake/windmill outputs. Physical release speeds were approximately 0.145–0.157 m/s for the preloads. Garden-batch release speeds were below 0.001 m/s: those shots used the stationary fallback. In the one 7/8 run, the fourth Blue preload encountered the already-tipping hive rather than completing an opening entry. These are six trials, not a statistical reliability guarantee or real-robot validation.

## Failure found and correction

The original PR code drove toward the garden but released **zero** balls in both baseline runs. Measured corrective yaw and changing image-derived target ranges were fed back into the desired heading, producing oscillation and repeatedly canceling feeder transport before a release. Both baseline runs timed out and parked with their preloads.

The correction keeps the aiming reference continuous in Pinpoint coordinates and plans with the desired turn rate. Actual measured motion and a fresh camera-derived pose remain mandatory in the independent release evaluation. No feed check was disabled to obtain the results. The rendered camera still rejects many shots during the far crossing; this is why the garden batch finishes stationary.

Motor/feeder acknowledgments matched all eight physical releases in every corrected trial. The separate `Moving feed events` telemetry can undercount because a load can be acknowledged during the internal feeder update; the reported moving-shot counts above come from the physics plant's actual launch speed and shot trails.

## Test conditions and provenance

- Standard reset: four robot preloads, original garden and flower inventory, original nectar-loaded hives. No practice extras, injected partner balls or injected robot poses.
- Team drive/Pinpoint `Constants`, `Intake` and predictive `CycleMechanisms`. The fixture disables the simulator's optional `Lab.prepare()` demo retuning. Shooter/Intake and team drive tuning sources were not changed.
- Existing simulator motor, drivetrain, ball, contact and hive dynamics. The local Physics/Settings/ContactMath source blobs match GitHub tree `0d6d1c5bf563799652e7aeb30ab9a9092761d233`; hashes are recorded with the results. The simulator checkout also contains unrelated pre-existing edits, which this test does not modify.
- Rendered RGB vision (`visionEngine=1`), 20 FPS, configured 60 ms latency, camera height 10 inches, forward offset 7 inches, pitch 43 degrees, fixed 86-degree launcher. Pixel tag detection and approximate tag range are simulator stand-ins, not a real Limelight camera.
- The isolated hardware adapter supplies the missing `transferServo` CRServo and maps its inward state to the existing plant gate. Required original tag artwork is copied into the isolated run folder. No Physics/Settings/ContactMath source changes or tuning adjustments are made.
- The runner samples actual state roughly every 100 ms and saves full JSON traces, shot trails and final telemetry before Stop resets the field. The committed CSV and shot-trail JSON include all six corrected trials.

## Reproduce

Use a BIOBUZZ-sim checkout with a built `build/active-build.txt`, bundled Java/JavaFX and libraries. From the robot repository:

```powershell
./tools/auton/run-moving-garden-sim.ps1 -SimulatorRoot 'C:/path/to/BIOBUZZ-sim' -Port 8782 -Repeats 2
```

The test creates an isolated folder under `build/`, compiles the production OpModes and fixture, starts a hidden local simulator, runs both alliances, and shuts down its own process. It rejects an occupied port. Full-cycle scoring checks require eight shots/entries, all four preloads during movement, four garden pickups, parking, no forbidden pickups, frame clearance and stopped outputs. A 7/8 trial fails those strict scoring checks, but all requested repetitions are still recorded. The `farMovingShots` field separately exposes the stationary fallback; passing a full-cycle score check does **not** mean the garden batch fired while moving.

The saved reusable runner itself was verified on the third Red/Blue pair. Android Java compilation, all 12,185 moving aim/route checks, and existing feeder/flywheel/vision contract checks also pass.

Evidence: [summary and source hashes](validation/moving-garden-summary.json), [sampled movement and controller gates](validation/moving-garden-traces.csv), [actual shot trajectories](validation/moving-garden-shot-trails.json). Physical field testing and further work on far-crossing camera visibility remain necessary.
