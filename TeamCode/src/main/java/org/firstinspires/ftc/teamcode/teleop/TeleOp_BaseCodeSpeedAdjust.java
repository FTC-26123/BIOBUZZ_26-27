package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Dual-controller TeleOp OpMode with dynamic shooter target speed adjustment.
 * Driver 1 controls Mecanum driving, drive speed switching, and AprilTag alignment.
 * Driver 2 controls intake, shooter, servo gate, preset speed switching,
 * and fine-tuning shooter target speed (+50 / -50 ticks/sec).
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

    @Override
    public void init() {
//        limelight.init(hardwareMap);
        aprilTagAlignment.init(limelight, TeleOpRuntime);
        driveTrain.init(hardwareMap, aprilTagAlignment);
        intake.init(hardwareMap);
        shooter.init(hardwareMap, intake);
        driveTrain.enablePositionHold(hardwareMap);
    }

    @Override
    public void start() {
        TeleOpRuntime.reset();
//        limelight.start();
    }

    @Override
    public void loop() {
        // Vision alignment & auto rotation
//        limelight.update();
        aprilTagAlignment.update();
        handleAutoRotation(gamepad1);

        // Driver 1 Drivetrain Control
        handleDrivetrainDpadSpeedSwitching(gamepad1);
        driveTrain.driveWithPositionHold(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, gamepad1.left_trigger > 0.3 || gamepad1.right_stick_button);

        // Driver 2 Intake Control
        handleManualIntakeControl(gamepad2);

        // Driver 2 Shooter Flywheel & Target Speed Adjustment (+50 / -50 ticks/s)
        handleManualShootingControl(gamepad2);
        handleShooterSpeedControl(gamepad2);
        shooter.update();

        // ---------------- Telemetry Status Output ----------------
        telemetry.addData("Drive Speed Multiplier (G1 Dpad Right)", driveTrain.getSpeedMultiplier());
        telemetry.addLine("---------------- Statuses ---------------");
        telemetry.addData("Shooter Status (G2 RT/RB)", shooter.getShooterStatus());
        telemetry.addData("Intake Status (G2 LT/Back/LB)", intake.getIntakeStatus());
        telemetry.addData("Target Launch Speed (+50 G2 DpadRight / -50 G2 DpadLeft)", "%.0f ticks/s", shooter.getTargetLaunchPower());
        telemetry.addData("Current Motor Speed", "%.0f ticks/s", shooter.getCurrentSpeed());
        telemetry.addData("Runtime (s)", "%.2f", TeleOpRuntime.seconds());
        telemetry.setMsTransmissionInterval(30);
        telemetry.addData("Position hold (hold G1 right stick button to bypass)", driveTrain.getPositionHoldStatus());
        telemetry.update();
    }

    @Override
    public void stop() {
        driveTrain.stop();
    }

    // ---------------------- Driver Input Handlers ----------------------

    private void handleManualShootingControl(Gamepad gamepad2) {
        if (gamepad2.right_trigger > 0.5) {
            shooter.start();
        } else if (gamepad2.right_bumper) {
            shooter.stop();
        }
    }

    private void handleShooterSpeedControl(Gamepad gamepad2) {
        // Cycle predefined speed presets on Dpad Up
        if (gamepad2.dpadUpWasPressed()) {
            shooter.switchSpeed();
        }
        // Increase target speed by 50 ticks/sec on Dpad Right
        if (gamepad2.dpadRightWasPressed()) {
            shooter.increaseTargetSpeed(50);
        }
        // Decrease target speed by 50 ticks/sec on Dpad Left
        if (gamepad2.dpadLeftWasPressed()) {
            shooter.decreaseTargetSpeed(50);
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

    private void handleAutoRotation(Gamepad gamepad1) {
        if (gamepad1.left_trigger > 0.3) {
            aprilTagAlignment.autoRotate();
        } else {
            aprilTagAlignment.setManualRotation();
        }
    }
}

