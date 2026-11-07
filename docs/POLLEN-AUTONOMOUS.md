# Complete basket autonomous programs

The Driver Station offers six programs under **26123 Flowerless**.

| Red alliance | Blue alliance | Behavior |
| --- | --- | --- |
| Red Basket 1 Shoot 4 and Park | Blue Basket 1 Shoot 4 and Park | Shoot four preloads and park. |
| Red Basket 1 Recover and Shoot | Blue Basket 1 Recover and Shoot | Shoot four, move forward, wait for the first tip, collect loose pollen, wait for the teammate to raise Basket 1, reshoot Basket 1, and park. |
| Red Park Only | Blue Park Only | Leave a separate wall mark and park using the drivetrain and Pinpoint. |

The scoring routines use floor pollen and exclude flower-contact areas from vision pickup. The previous menu entries have their Autonomous annotations removed. Each current entry is a full autonomous OpMode selected and started once.

A confirmed four-shot opening is required before recovery. The first tip is checked using the lowered Basket 1 tag height for 0.65 seconds. The second volley requires raised Basket 1 observations for 0.65 seconds, fresh Limelight shooting range, and a valid 86-degree trajectory. The recovery program holds its shots while waiting for the teammate and yields to the measured parking reserve. The hard scoring deadline is 29.7 seconds.

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
| Limelight | limelight | AprilTag shooting pipeline 1, neural pollen pipeline 7 |
| Logitech USB webcam | Webcam 1 | FTC VisionPortal and AprilTagProcessor |

Collection runs the roller and transfer CRServo while holding the windmill. A permitted shot adds windmill feed. Cancellation, completed pulses, parking, and Stop clear feed outputs. Flywheel load events acknowledge estimated ball releases.

Park Only initializes the drivetrain and Pinpoint. The six scoring programs also initialize the mechanisms and cameras.

## Vision and calibration

Configure the Limelight neural detector with the exact label yellow_pollen on pipeline 7. The robot uses projected detector rays and odometry to find pollen. Ground projection assumes 0.305 m camera height, +10 degrees pitch, and zero yaw, roll, or horizontal offset. Match the physical mount or calibrate these values in SpillCollector.

Configure Limelight AprilTag pipeline 1 for the field's printed tags, with 3D target pose enabled and the actual 3.25-inch tag size. Autonomous switches to pipeline 1 during shooting and returns to pipeline 7 for collection. The webcam remains a separate basket interlock.

Shooting power uses the expected basket's fresh Limelight target pose in camera space. Limelight camera axes are right, down, and forward. The code removes camera pitch, accounts for the individual tag's board offset, and compensates for capture age using Pinpoint velocity, acceleration and yaw rate. It supplies the resulting measured shooting position to the 86-degree moving-shot solver. Pinpoint continues to drive the path. These axes follow [Limelight's coordinate system documentation](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems).

Missing, stale, wrong-pipeline, wrong-basket, lowered-basket, or inconsistent measurements block feeding. An estimate more than 0.45 m from Pinpoint is rejected. Telemetry reports the measured basket range, measurement status, and 86-degree shooting angle. LimelightShotPose uses the same physical mount constants as SpillCollector.

The Logitech webcam independently verifies basket tags. Basket 1 uses red tags 34–37 or blue 42–45. Basket 2 uses red 30–33 or blue 38–41. Team routines use the tag pose and Pinpoint distance to check that the assigned basket is raised for 0.65 seconds before feeding. That height check assumes the webcam mount in WebcamTags, currently 0.305 m high and pitched +10 degrees. Calibrate the camera intrinsics and physical mount before field use.

The shooting solver uses the measured 86-degree elevation. Muzzle height, projectile drag and slip, flywheel response, feeder travel, motor encoder scale, and load thresholds need physical measurements. Transfer startup delay is currently estimated at 35 ms. Recalibrate transport timing for the current windmill and CRServo hardware. The single-ball pulse controller uses intake encoder travel and flywheel load feedback.

Each Team role pursues pollen within its own alliance-normalized quadrant. Pickup approaches and parking route around the whole metal frame with the existing swept-body allowance. Establish field dimensions and tracking margin on the physical robot.

## Verification scope

The FTC Android app build checks the six registered OpModes and their SDK/Pedro interfaces. Standalone feeder, flywheel, and vision contract checks run through tools/auton/check-models.ps1.

The flowerless routes have 66 ordering, four-shot, starting-mark, and frame-bypass checks, plus 64 raised/lowered basket-height checks. The route helpers come from the simulator basket lineup. Simulator role tests used an injected teammate batch to exercise basket availability. Simulated shot counts and tips describe that field model. Forty-four additional fixture checks cover Limelight projection, both alliances and baskets, capture-age compensation, invalid observations, and measured range changing the calculated shooting speed. The updated Limelight shooting flow and the real robot's CRServo feed still require physical autonomous trials.

The previous basket wrappers have their registration annotations removed. The six flowerless OpModes are the selectable autonomous lineup.
