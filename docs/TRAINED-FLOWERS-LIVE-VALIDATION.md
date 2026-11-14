# Actual simulator validation



Simulator: https://github.com/d-mankotia/BIOBUZZ-sim, commit `0d6d1c5bf563799652e7aeb30ab9a9092761d233`, freshly cloned into this isolated work folder. Actual current Java TrainedFlowerAuto, Pedro/Foresight, Constants, Shooter, Intake and FlowerArm execute against simulator physics; this is not a scripted pose-state substitute.



Tested TrainedFlowerAuto SHA256: `f41d9707c128b22f9a13136f9df1e3dcc344b6b4a4dd6a77c3adf783ad8140de`. The parent subsequently added finite-velocity fault handling; these successful runs have finite velocity, so that guard does not change their behavior. Default selected settings: forward power .65, timed entry .90 seconds, heading correction P1.5/clamp.25, shooter1450 with .25-second recovery, robot-right3-inch lineup, shooting marks moved1inch backward along aim. No demo drivetrain retuning.



## Measured results



| Scenario and alliance | Park clock(s) | Launches | Hits | Flower balls | Own tips | Result |

|---|---:|---:|---:|---:|---:|---|

| p0.55-t0.9-Blue | 22.693 | 12 | 11 | 8 | 2 | Parked |

| p0.55-t0.9-Red | 22.719 | 12 | 11 | 8 | 2 | Parked |

| p0.65-t0.9-Blue | 22.710 | 12 | 11 | 8 | 2 | Parked |

| p0.65-t0.9-lowgrip12v-Blue | 22.805 | 12 | 11 | 8 | 2 | Parked |

| p0.65-t0.9-lowgrip12v-Red | 22.840 | 12 | 11 | 8 | 2 | Parked |

| p0.65-t0.9-lowvoltage10v5-Blue | 23.552 | 12 | 11 | 8 | 2 | Parked |

| p0.65-t0.9-lowvoltage10v5-Red | 23.641 | 12 | 11 | 8 | 2 | Parked |

| p0.65-t0.9-Red | 22.706 | 12 | 11 | 8 | 2 | Parked |

| p0.75-t0.9-Blue | 22.655 | 12 | 11 | 8 | 2 | Parked |

| p0.75-t0.9-Red | 22.804 | 12 | 11 | 8 | 2 | Parked |



All ten runs finished both flower entries by timer, collected eight flower balls, completed three batches and parked. Hits were11/12, not perfect12/12. Garden and nectar collection remained0. No runtime parking reason or abort was reported.



The two baseline runs without heading correction failed at the first pot: red15.192degrees yaw/elapsed6.093s and blue15.507degrees/5.961s triggered the clearance interlock after collecting only1flower ball. Thus the revised success is not inferred only from source inspection; it fixes an observed live failure.



## Timing and scenarios



Profiles use the supported /api/profile apply operation. Nominal physical settings:12V, gripForward.95, gripStrafe.72. The lowgrip12v scenario uses12V/.8/.6; the lowvoltage10v5 scenario uses10.5V with default grip. The10.5V trace JSONs include the server-returned applied settings. Earlier cases follow the identical apply code but predate the added metadata field. Final persisted profile also confirms the applied values.



Every entry reported `Timed flower push complete`. With100ms API polling and asynchronous cached telemetry, first-observed PUSH to first-observed COLLECT durations ranged 0.753 to 1.036s; these are sampled transitions, not exact motor-on timestamps. Source clock and state tests enforce the .90-second cutoff; the live trace establishes that no normal position/yaw cutoff terminated entries.



## Required SDK adapters and limits



The specified simulator checkout lacks the current team's transfer motor and CR flower-arm/limit switches. An isolated BioBuzzBot adapter adds a dynamic motor named transfer, maps measured inward transfer motion to the existing physical feed gate, adds the commanded CR arm and kinematic up/down switches. It does not inject balls, scores, poses, tips or shot counts. Arm travel rate is a fixture estimate, not a real arm calibration. The simulator already represents flower pickup and chassis/pot collision, but does NOT model the long thin ramp's collision or flex.



Simulator chassis: length.4576660022m, width.400591312m, bodyForward.02483252995m, bodyRight.000349899754m; origin-to-front9.987in, rear8.031in and halfwidth7.886in. Flower throat requires longitudinal distance.15 to .34m and lateral<.17m. These are simulator values, not measured robot dimensions.



RobotSettings.ensureLoaded is stubbed only to avoid loading absent saved robot preferences; source defaults are used. Lab.simulationSetup is disabled, so it cannot overwrite actual team Constants. Real robot saved PID/mechanism settings can differ. The simulator SDK voltage sensor reports14V even when physical batteryVoltage is12V or10.5V; this compensation mismatch is a simulator limitation and is preserved in these tests.



This matrix does not exhaust all possible initial placement, sensor faults, friction or battery values. Parent state/geometry tests cover delayed readiness and cancellation cases separately. Real-field success, real ramp impact force and real pickup cannot be guaranteed by these tests.



## Reproduction



Portable helpers are in `tools/auton/live-sim/` in the robot repository. Follow its README with a clean isolated simulator clone, JDK21+, Python3.10+ and Node18+. Raw trace JSONs remain beside this report. `summarize.py` extracts metrics using the last telemetry-bearing row, because the match clock resets world state at shutdown.


## Final exact-source portable smoke

The portable prepare/build/launch workflow was exercised with the exact production source including finite-velocity checking. Both source files have SHA256 `838c9142ad7ff08e346888f77e040a98ec73c95f5084f9ec1cff5bf37b8f81f9`. These two runs bring the successful live scenario total to12.

| Exact-source alliance | Park clock(s) | Launches | Hits | Flower balls | Own tips | Result |
|---|---:|---:|---:|---:|---:|---|
| Blue | 24.173 | 12 | 11 | 8 | 2 | Parked |
| Red | 22.795 | 12 | 11 | 8 | 2 | Parked |
