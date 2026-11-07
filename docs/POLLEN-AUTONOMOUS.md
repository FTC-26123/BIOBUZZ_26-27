# Complete basket autonomous programs

The Driver Station offers seven programs under **26123 Match Lineup**.

| Red alliance | Blue alliance | Behavior |
| --- | --- | --- |
| Red Basket 1 | Blue Basket 1 | Open the initially raised basket, collect flower and garden pollen, run scoring transfers, and park. |
| Red Basket 1 Team | Blue Basket 1 Team | Stay in the opening scoring quadrant, recover fallen pollen, use local garden pollen when needed, and shoot again when the assigned basket rises. |
| Red Basket 2 Team | Blue Basket 2 Team | Wait for the other robot to tip Basket 1, shoot into Basket 2, recover local pollen or use the local pot, and repeat while time permits. |

**Park Only** is one program for both alliances. After Init, X selects Blue and B selects Red. Default is Red. Place the robot at the matching Basket 1 start before Start.

Every program is a complete autonomous OpMode. The scoring programs run preload shots, acquisition, shooting, repeat collection, and parking after one press of Start. Team programs reserve separate loading-zone stops and leave the scoring quadrant when they return to park. The measured return-path budget preempts optional work. A hard 29.7-second deadline stops outputs.

See the [official CAD setup map and starting coordinates](AUTONOMOUS-LINEUP.md).

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

The FTC Android app build checks the seven registered OpModes and their SDK/Pedro interfaces. Standalone feeder, flywheel, and vision contract checks run through tools/auton/check-models.ps1.

The route logic comes from the tested simulator basket lineup. Simulator role tests used an injected teammate batch to exercise basket availability. Simulated shot counts and tips describe that field model. Forty-four additional fixture checks cover Limelight projection, both alliances and baskets, capture-age compensation, invalid observations, and measured range changing the calculated shooting speed. The updated Limelight shooting flow and the real robot's CRServo feed still require physical autonomous trials.

The retired autonomous wrappers and their unused legacy HiveAim controller have been removed. The seven requested OpModes are the complete selectable autonomous lineup.
