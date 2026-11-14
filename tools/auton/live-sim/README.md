# Live trained-auton verification

Use a **new isolated clone**, not the simulator installation you play with. Preparation modifies its imported code/hardware adapter and moves its old imported TeleOp sources into a recoverable `legacy-teleop` folder (their old Shooter API is incompatible with current code).

Requirements: JDK21+ on JAVA_HOME, Python3.10+, Node18+, Windows desktop with JavaFX support. From the robot repository:

```powershell
git clone --depth 1 https://github.com/d-mankotia/BIOBUZZ-sim.git build/live-sim-checkout
git -C build/live-sim-checkout fetch origin 0d6d1c5bf563799652e7aeb30ab9a9092761d233
git -C build/live-sim-checkout checkout --detach 0d6d1c5bf563799652e7aeb30ab9a9092761d233
python tools/auton/live-sim/prepare-simulator.py build/live-sim-checkout .
Push-Location ./build/live-sim-checkout
& ./Rebuild.ps1
Pop-Location
& ./tools/auton/live-sim/run-live.ps1 -SimulatorRoot ./build/live-sim-checkout
& ./tools/auton/live-sim/run-live.ps1 -SimulatorRoot ./build/live-sim-checkout -Scenario lowgrip12v
& ./tools/auton/live-sim/run-live.ps1 -SimulatorRoot ./build/live-sim-checkout -Scenario lowvoltage10v5
```

The runner launches a hidden simulator, runs red and blue for a full match, saves actual physics/state/telemetry JSON, and stops the process. Results default to `build/live-sim-results`, with unique timestamp labels. Override `-ResultsDirectory`, `-RunTag`, `-Port`, `-Power` or `-Seconds` if needed. Inputs default to .65 power/.90seconds. Failed launch-count, flower-count, hive-tip or parking checks return failure.

Preparation adds SDK hardware missing from this simulator: the team's motor transfer, CR flower arm, and kinematic touch switches. It uses source-default robot preferences, disables simulator demo tuning, and does not inject ball counts, pose, tips or scoring. The simulator has no full thin-ramp collision model; arm dynamics are not calibrated, and its SDK battery sensor stays14V even when profile physical battery differs. Test traces validate this simulator, not physical contact safety or real robot guarantees.

Preparation refuses a simulator SHA other than the tested commit and verifies hardware-adapter replacement anchors. Review the fixture before supporting a newer simulator revision.
