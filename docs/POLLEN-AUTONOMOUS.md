# Complete basket autonomous programs

Two additional programs, **Red Moving Shots + Garden + Hive** and **Blue Moving Shots + Garden + Hive**, now drive toward the garden while shooting, strafe for four garden balls, and shoot toward the opposite hive. They require Limelight and the predictive mechanism helper. See [moving garden setup and verification limits](MOVING-GARDEN.md). The eight earlier programs described below retain their existing behavior.

The Driver Station offers eight programs under **26123 Flowerless**.

| Red alliance | Blue alliance | Behavior |
| --- | --- | --- |
| Red Basket 1 Shoot 4 and Park | Blue Basket 1 Shoot 4 and Park | Shoot four preloads and park. |
| Red Park Only | Blue Park Only | Leave a separate wall mark and park using the drivetrain and Pinpoint. |
| Red Shoot 4 + Garden and Park | Blue Shoot 4 + Garden and Park | Shoot the opening, collect our garden and park with pollen for TeleOp. |
| Red Basket 2 Wait 15 Shoot 4 and Park | Blue Basket 2 Wait 15 Shoot 4 and Park | Opposite-wall start; wait/aim/spin up, feed four after 15 seconds, then park. Teammate must raise Basket 2. |

The eight selectable programs are flowerless. Recovery programs are unregistered because the only camera detects AprilTags, not pollen. Previous flower and recovery source remains available but is absent from the autonomous menu.

The existing Basket 1 shooting routines call the team's `Shooter` and `Intake` classes through their normal update methods. Shooter's default preset is 1500 encoder ticks/second. Its existing PIDF, battery compensation and speed hysteresis control the flywheel and feeding. Those Basket 1 routines start it only after arriving at the shooting pose; movement and parking keep the mechanisms stopped. The delayed Basket 2 pair instead spins up with feeders off during its approach and wait. Four estimated releases or a 14-second timeout begin parking, with a 29.7-second hard deadline.

See the [official CAD setup map and starting coordinates](AUTONOMOUS-LINEUP.md). Park Only uses a different wall mark from the shooting routines and the former Basket 2 start.

## Robot configuration

Use the team's tuned org.firstinspires.ftc.teamcode.pedro.Constants follower and Pinpoint configuration.

| Device | Configuration name | Autonomous command |
| --- | --- | --- |
| Front/back left drive | frontLeftMotor, backLeftMotor | REVERSE |
| Front/back right drive | frontRightMotor, backRightMotor | FORWARD |
| Pinpoint | pinpoint | Team offsets and pod directions |
| Shooter motor | shooter | FORWARD, positive target velocity, existing Shooter software PIDF in RUN_WITHOUT_ENCODER |
| Intake motor | intake | REVERSE, negative inward power |
| Windmill motor | windmill | FORWARD, positive power during permitted shooting pulses |
| Transfer CRServo | transferServo | Team Intake.TRANSFER_SERVO_POWER, currently -1 inward |

Collection runs the roller and transfer CRServo while holding the windmill. A permitted shot adds windmill feed. Cancellation, completed pulses, parking, and Stop clear feed outputs. Flywheel load events acknowledge estimated ball releases.

Park Only initializes the drivetrain and Pinpoint. The six shooting programs also initialize the mechanisms. None of the eight programs initializes a Limelight or webcam.

## Fixed-position shooting

Red shoots from visualizer (57.0146, 27.4806) inches. Blue shoots from (84.4854, 114.0194), using a 180-degree alliance rotation. The robot's field-centered positions are (-13.7354, -43.2694) and (+13.7354, +43.2694) inches. This moves every active shooting routine exactly one inch backwards along its aim line, away from the hive. The fixed heading points at Basket 1. See the lineup guide for starts and setup map.

The 86-degree launch angle is a physical configuration. Autonomous does not use a ballistic solver, tag range, or camera mount to choose shooting speed. It uses Shooter's existing 1500 ticks/second default. Tune the subsystem on the robot as usual.

`PreloadShotCounter` estimates release events using intake encoder transport, a previously recovered flywheel, a speed dip and minimum spacing. It cannot guarantee physical inventory without a ball sensor. No detected load does not count as a ball. Sustained low speed cannot repeatedly count the same dip. Stop, parking, lost shooting position, localization faults and timeouts stop Shooter and all Intake outputs.

Legacy unregistered autonomous helpers remain in the repository; the eight menu entries use `FixedShotParkAuto`, `GardenParkAuto`, `DelayedBasketTwoAuto` or `ParkOnlyAuto`, with no camera initialization.

## Teammate parking

Blue Basket 2 Wait 15 parks directly at visualizer (131.44, 19.94), reserving the nearer slot (131.77, 45.55) for its teammate. The other seven active autons use the near alliance slot and reserve the farther slot for a teammate. See the lineup map and coordinate table. Slot centers are 0.55 m apart, leaving 0.0928 m (3.65 inches) between two 18-inch footprints. Both shooting families now use the mark one inch farther from the hive; shooter speed is unchanged. This reserves physical space; coordinate the other robot's route and parking eligibility with the alliance.

## Verification scope

Wall pickup returns retreat to a 1.42 m inset while retaining the pickup heading, then turn before continuing. Wall approaches complete their turn inboard. Intermediate clearance waypoints require position, heading, and low speed before advancing. `tools/auton/PathClearanceChecks.java` samples the swept 18-inch chassis along both alliances' stage and parking paths, including a grid of wall pickup poses. These geometric checks assume the configured chassis size and do not model tracking error or another robot.

The FTC Android app build checks the eight registered autonomous OpModes and their SDK/Pedro interfaces. Standalone feeder, flywheel, and vision contract checks run through tools/auton/check-models.ps1.

The 5,234 teammate-parking checks sample both active route families against a stationary 18-inch teammate in the reserved slot, verify mirrored centers, and require at least 9 cm between the parked footprints.

The 23 fixed-shot checks cover the one-inch backwards offset, exact Blue rotation, stationary shooting stage, parking, and release counting under feeding, no-load and stalled-transport scenarios. Updated swept-chassis checks pass. The Android build compiles against the latest main-branch Shooter and Intake implementation.

Only Red/Blue Basket 1 Shoot 4 and Park, Red/Blue Shoot 4 + Garden and Park, Red/Blue Basket 2 Wait 15 Shoot 4 and Park, and Red/Blue Park Only are registered. Driver TeleOp, Intake and Pedro constants are unchanged. Shooter has an additive feeder-off `updateSpinupOnly()` method for the delayed routines; its normal update and tuning are unchanged. See [garden verification](GARDEN-PARK.md) for actual OpMode/follower trials and exported Pedro projects. Physical field trials are required.

For the delayed routines, use the [Basket 2 wall starts and teammate setup](BASKET-TWO-WAIT15.md). Their shooting distance matches the current Basket 1 mark. They spin up during the wait, with all feeder outputs off until at least 15 seconds.
