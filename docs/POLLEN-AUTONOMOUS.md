# Complete basket autonomous programs

The Driver Station offers four programs under **26123 Flowerless**.

| Red alliance | Blue alliance | Behavior |
| --- | --- | --- |
| Red Basket 1 Shoot 4 and Park | Blue Basket 1 Shoot 4 and Park | Shoot four preloads and park. |
| Red Park Only | Blue Park Only | Leave a separate wall mark and park using the drivetrain and Pinpoint. |

The four selectable programs are flowerless. Recovery programs are unregistered because the only camera detects AprilTags, not pollen. Previous flower and recovery source remains available but is absent from the autonomous menu.

The fixed shooting routines call the current team's `Shooter` and `Intake` classes without changing either subsystem. Shooter's default preset is 1500 encoder ticks/second. Its existing PIDF, battery compensation and speed hysteresis control the flywheel and feeding. Autonomous starts it only after arriving at the shooting pose; movement and parking keep the mechanisms stopped. Four estimated releases or a 10-second timeout begin parking, with a 29.7-second hard deadline.

See the [official CAD setup map and starting coordinates](AUTONOMOUS-LINEUP.md). Park Only uses a different wall mark from the shooting routines and the former Basket 2 start.

## Robot configuration

Use the team's tuned org.firstinspires.ftc.teamcode.pedro.Constants follower and Pinpoint configuration.

| Device | Configuration name | Autonomous command |
| --- | --- | --- |
| Front/back left drive | frontLeftMotor, backLeftMotor | REVERSE |
| Front/back right drive | frontRightMotor, backRightMotor | FORWARD |
| Pinpoint | pinpoint | Team offsets and pod directions |
| Shooter motor | shooter | FORWARD, positive encoder velocity, RUN_USING_ENCODER |
| Intake motor | intake | REVERSE, negative inward power |
| Windmill motor | windmill | FORWARD, positive power during permitted shooting pulses |
| Transfer CRServo | transferServo | Team Intake.TRANSFER_SERVO_POWER, currently -1 inward |

Collection runs the roller and transfer CRServo while holding the windmill. A permitted shot adds windmill feed. Cancellation, completed pulses, parking, and Stop clear feed outputs. Flywheel load events acknowledge estimated ball releases.

Park Only initializes the drivetrain and Pinpoint. The two shooting programs also initialize the mechanisms. None of the four programs initializes a Limelight or webcam.

## Fixed-position shooting

Red shoots from visualizer (57.05, 28.48) inches. Blue shoots from (84.45, 113.02), using a 180-degree alliance rotation. The robot's field-centered positions are (-13.70, -42.27) and (+13.70, +42.27) inches. The fixed heading points at Basket 1. See the lineup guide for starts and setup map.

The 86-degree launch angle is a physical configuration. Autonomous does not use a ballistic solver, tag range, or camera mount to choose shooting speed. It uses Shooter's existing 1500 ticks/second default. Tune the subsystem on the robot as usual.

`PreloadShotCounter` estimates release events using intake encoder transport, a previously recovered flywheel, a speed dip and minimum spacing. It cannot guarantee physical inventory without a ball sensor. No detected load does not count as a ball. Sustained low speed cannot repeatedly count the same dip. Stop, parking, lost shooting position, localization faults and timeouts stop Shooter and all Intake outputs.

Legacy unregistered autonomous helpers remain in the repository; the four menu entries use `FixedShotParkAuto` or `ParkOnlyAuto`, with no camera initialization.

## Teammate parking

All four active autons park in the near alliance slot rather than the middle of the loading area. A farther slot is reserved for a teammate. See the lineup map and coordinate table. Slot centers are 0.55 m apart, leaving 0.0928 m (3.65 inches) between two 18-inch footprints. The fixed shot mark and shooter speed are unchanged. This reserves physical space; coordinate the other robot's route and parking eligibility with the alliance.

## Verification scope

Wall pickup returns retreat to a 1.42 m inset while retaining the pickup heading, then turn before continuing. Wall approaches complete their turn inboard. Intermediate clearance waypoints require position, heading, and low speed before advancing. `tools/auton/PathClearanceChecks.java` samples the swept 18-inch chassis along both alliances' stage and parking paths, including a grid of wall pickup poses. These geometric checks assume the configured chassis size and do not model tracking error or another robot.

The FTC Android app build checks the four registered autonomous OpModes and their SDK/Pedro interfaces. Standalone feeder, flywheel, and vision contract checks run through tools/auton/check-models.ps1.

The 5,234 teammate-parking checks sample both active route families against a stationary 18-inch teammate in the reserved slot, verify mirrored centers, and require at least 9 cm between the parked footprints.

The 21 fixed-shot checks cover the requested Red coordinates, exact Blue rotation, stationary shooting stage, parking, and release counting under feeding, no-load and stalled-transport scenarios. Updated swept-chassis checks pass. The Android build compiles against the latest main-branch Shooter and Intake implementation.

Only Red/Blue Basket 1 Shoot 4 and Park and Red/Blue Park Only are registered. Driver TeleOp, Shooter, Intake and Pedro constants are unchanged relative to the latest main branch. Physical field trials are required.
