# Complete basket autonomous programs

The Driver Station offers four programs under **26123 Flowerless**.

| Red alliance | Blue alliance | Behavior |
| --- | --- | --- |
| Red Basket 1 Shoot 4 and Park | Blue Basket 1 Shoot 4 and Park | Shoot four preloads and park. |
| Red Park Only | Blue Park Only | Leave a separate wall mark and park using the drivetrain and Pinpoint. |

The four selectable programs are flowerless. Recovery programs are unregistered because the only camera detects AprilTags, not pollen. Previous flower and recovery source remains available but is absent from the autonomous menu.

A confirmed four-shot opening is required before the shooting program parks. Feeding requires Basket 1 raised observations for 0.65 seconds, a fresh expected-basket Limelight pose, and a valid 86-degree trajectory. Missing vision yields to the parking reserve. The scoring deadline is 29.7 seconds.

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
| Limelight | limelight | AprilTag pipeline 1 throughout autonomous |

Collection runs the roller and transfer CRServo while holding the windmill. A permitted shot adds windmill feed. Cancellation, completed pulses, parking, and Stop clear feed outputs. Flywheel load events acknowledge estimated ball releases.

Park Only initializes the drivetrain and Pinpoint. The two shooting programs also initialize the mechanisms and Limelight. Neither Park Only program requires a camera.

## Vision and calibration

There is no webcam. Configure Limelight AprilTag pipeline 1 for the field's printed tags, with 3D target pose enabled and the actual 3.25-inch tag size. The active autonomous programs never request neural pipeline 7.

`LimelightMount` records the approximate measured lens height of 10 inches (0.254 m), forward offset of 7 inches (0.1778 m) from the Pinpoint pose origin, zero lateral offset, and upward pitch of 43 degrees. Confirm these physical measurements before tuning shots. The camera pitch is separate from the ball's 86-degree launch elevation.

Shooting uses the expected basket's fresh target pose in camera space. The code removes camera pitch, subtracts the rotated camera-to-origin translation, accounts for each tag's board offset, and compensates for capture age using Pinpoint velocity, acceleration, and yaw rate. Pinpoint continues to drive the paths. These axes follow [Limelight's coordinate documentation](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems).

Limelight also verifies the basket's height before feeding. Basket 1 uses red tags 34–37 or blue 42–45. Missing, stale, wrong-pipeline, wrong-basket, lowered, or inconsistent observations block feeding. Estimates more than 0.45 m from Pinpoint are rejected. Configure and use **26123 Limelight AprilTag Check** for a camera-only diagnostic that commands no motors.

The shooting solver uses the measured 86-degree elevation. Muzzle height, projectile drag and slip, flywheel response, feeder travel, motor encoder scale, and load thresholds need physical measurements. Transfer startup delay is currently estimated at 35 ms. Recalibrate transport timing for the current windmill and CRServo hardware. The single-ball pulse controller uses intake encoder travel and flywheel load feedback.

Each Team role pursues pollen within its own alliance-normalized quadrant. Pickup approaches and parking route around the whole metal frame with the existing swept-body allowance. Establish field dimensions and tracking margin on the physical robot.

## Verification scope

Wall pickup returns retreat to a 1.42 m inset while retaining the pickup heading, then turn before continuing. Wall approaches complete their turn inboard. Intermediate clearance waypoints require position, heading, and low speed before advancing. `tools/auton/PathClearanceChecks.java` samples the swept 18-inch chassis along both alliances' stage and parking paths, including a grid of wall pickup poses. These geometric checks assume the configured chassis size and do not model tracking error or another robot.

The FTC Android app build checks the four registered autonomous OpModes and their SDK/Pedro interfaces. Standalone feeder, flywheel, and vision contract checks run through tools/auton/check-models.ps1.

The 53 synthetic Limelight mount checks cover 43-degree pitch, 10-inch height, 7-inch forward translation, both alliances and baskets, capture latency, acceleration, raised/lowered observations, invalid frames, and range changing shooting power. The swept-chassis path checks and Android build also pass. These are software and geometry checks; physical autonomous trials remain required.

Only Red/Blue Basket 1 Shoot 4 and Park and Red/Blue Park Only are registered. Driver TeleOp and Pedro constants are unchanged.
