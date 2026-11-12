package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystems.AutoShot;
import org.firstinspires.ftc.teamcode.subsystems.FlowerArm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Dual-controller TeleOp OpMode with dynamic shooter target speed adjustment.
 * Driver 1 controls Mecanum driving, drive speed switching, and AprilTag alignment.
 * Driver 2 controls intake, shooter, servo gate, preset speed switching,
 * and trimming the shot speed of each preset.
 */
@TeleOp
public class TeleOp_BaseCodeSpeedAdjust extends OpMode {

    // ---------------------- Subsystems & Hardware Objects ----------------------
    public Shooter shooter = new Shooter();
    public Intake intake = new Intake();
    public MecanumDrive driveTrain = new MecanumDrive();
    public ElapsedTime TeleOpRuntime = new ElapsedTime();
    public Limelight limelight = new Limelight();
    public AprilTagAlignment aprilTagAlignment = new AprilTagAlignment();
    /** Camera aiming + distance-based shot speed. Does nothing until ShotConfig.AUTO_SHOT_ENABLED is true. */
    public AutoShot autoShot = new AutoShot();
    /** Servo arm for the flower bed. Does nothing until FlowerArm.ENABLED is true. */
    public FlowerArm flowerArm = new FlowerArm();

    @Override
    public void init() {
        aprilTagAlignment.init(limelight, TeleOpRuntime);
        autoShot.init(hardwareMap, limelight, aprilTagAlignment, shooter, TeleOpRuntime);
        driveTrain.init(hardwareMap, aprilTagAlignment);
        intake.init(hardwareMap);
        flowerArm.init(hardwareMap);   // does not move the arm
        shooter.init(hardwareMap, intake);
        driveTrain.enablePositionHold(hardwareMap);
    }

    /**
     * While waiting for Play: pick the alliance for camera aiming (gamepad 1 X = blue, B = red).
     */
    @Override
    public void init_loop() {
        autoShot.selectAlliance(gamepad1, telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        TeleOpRuntime.reset();
        autoShot.start();
    }

    @Override
    public void loop() {
        // Vision alignment & auto rotation
        boolean aim = aimButton();
        autoShot.update(aim);
        aprilTagAlignment.update();
        handleAutoRotation(aim);

        // Driver 1 Drivetrain Control
        handleDrivetrainDpadSpeedSwitching(gamepad1);
        driveTrain.driveWithPositionHold(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, aim || gamepad1.right_stick_button);

        // Driver 2 Intake Control
        handleManualIntakeControl(gamepad2);

        // Driver 2 Shooter Flywheel & shot preset / trim
        handleManualShootingControl(gamepad2);
        handleShooterSpeedControl(gamepad2);
        shooter.update();
        handleFlowerArm();

        // ---------------- Telemetry Status Output ----------------
        telemetry.addData("Drive Speed Multiplier (G1 Dpad Right)", driveTrain.getSpeedMultiplier());
        telemetry.addLine("---------------- Statuses ---------------");
        telemetry.addData("Shooter (G2 hold RT fire / Y spin up / RB off)", shooter.getShooterStatus());
        telemetry.addData("Intake Status (G2 LT/Back/LB)", intake.getIntakeStatus());
        telemetry.addData("Shot preset (Dpad Up next, Dpad Down prev, Dpad Right/Dpad Left +/-" + (int) Shooter.TRIM_STEP
                + ", X clear)", shooter.getPresetText());
        telemetry.addData("Flywheel", "%.0f of %.0f ticks/s  %s", shooter.getLastSpeed(),
                shooter.getTargetLaunchPower(), shooter.isReady() ? "READY" : shooter.isRunning() ? "..." : "off");
        if (intake.isJammed()) telemetry.addLine("!! FEED JAMMED - eject with Back, then fire again !!");
        flowerArm.addTelemetry(telemetry, "G2 A down / B up");
        autoShot.addTelemetry(telemetry);
        telemetry.addData("Runtime (s)", "%.2f", TeleOpRuntime.seconds());
        telemetry.setMsTransmissionInterval(30);
        telemetry.addData("Position hold (hold G1 right stick button to bypass)", driveTrain.getPositionHoldStatus());
        telemetry.update();
    }

    @Override
    public void stop() {
        driveTrain.stop();
        autoShot.stop();
    }

    // ---------------------- Driver Input Handlers ----------------------

    /**
     * Shooter: HOLD the right trigger to fire (balls feed one at a time, each only when the flywheel is at
     * speed). Releasing it stops feeding but keeps the flywheel spinning for the next volley. Y spins the
     * flywheel up without feeding. Right bumper turns the flywheel off.
     */
    private void handleManualShootingControl(Gamepad gamepad2) {
        if (gamepad2.right_bumper) {
            shooter.stop();
        } else if (gamepad2.right_trigger > 0.5) {
            shooter.start();
        } else {
            shooter.holdFire();   // does nothing unless the shooter was firing
        }
        if (gamepad2.yWasPressed()) {
            shooter.spinUp();
        }
    }

    /**
     * Shot power: Dpad Up = next preset, Dpad Down = previous preset,
     * Dpad Right / Dpad Left = +/- {@link Shooter#TRIM_STEP} ticks/sec on the current preset, X = clear that trim.
     * The trim stays with its preset, so a correction made at one shooting spot is still there next time.
     */
    private void handleShooterSpeedControl(Gamepad gamepad2) {
        if (gamepad2.dpadUpWasPressed()) {
            shooter.switchSpeed();
        }
        if (gamepad2.dpadDownWasPressed()) {
            shooter.previousSpeed();
        }
        if (gamepad2.dpadRightWasPressed()) {
            shooter.increaseTargetSpeed(Shooter.TRIM_STEP);
        }
        if (gamepad2.dpadLeftWasPressed()) {
            shooter.decreaseTargetSpeed(Shooter.TRIM_STEP);
        }
        if (gamepad2.xWasPressed()) {
            shooter.resetTrim();
        }
    }

    private void handleManualIntakeControl(Gamepad gamepad2) {
        if (gamepad2.left_trigger > 0.5) {
            intake.start();
        } else if (gamepad2.back) {
            intake.out();
        } else if (gamepad2.left_bumper) {
            intake.stop();
        }
    }

    private void handleDrivetrainDpadSpeedSwitching(Gamepad gamepad1) {
        if (gamepad1.dpadRightWasPressed()) {
            driveTrain.switchSpeed();
        }
    }

    /**
     * FLOWER ARM. A = arm DOWN (then drive into the flower bed with the intake running), B = arm UP.
     *
     * TO CHANGE THE BUTTONS: edit the two lines below, e.g. gamepad2.guideWasPressed(),
     * and the label in the telemetry line.
     */
    private void handleFlowerArm() {
        if (gamepad2.aWasPressed()) {
            flowerArm.down();
        }
        if (gamepad2.bWasPressed()) {
            flowerArm.up();
        }
    }

    /**
     * AIM BUTTON. Hold it to turn the robot toward our basket and set the shot speed from the distance.
     * It also bypasses the position hold so the robot is free to turn.
     *
     * TO CHANGE THE BUTTON: edit the one line below. Examples:
     *   gamepad1.left_trigger > 0.3     (current: driver's left trigger)
     *   gamepad1.left_stick_button      (click the left stick)
     *   gamepad2.left_stick_button      (let the operator aim instead)
     * Then update the "AIM" label in the telemetry line if you want the screen to match.
     */
    private boolean aimButton() {
        return gamepad1.left_trigger > 0.3;
    }

    private void handleAutoRotation(boolean aim) {
        if (aim) {
            aprilTagAlignment.autoRotate();
        } else {
            aprilTagAlignment.setManualRotation();
        }
    }
}

