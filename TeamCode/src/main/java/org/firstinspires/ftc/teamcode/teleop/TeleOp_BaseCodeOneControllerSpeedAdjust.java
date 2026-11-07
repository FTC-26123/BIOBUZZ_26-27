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
 * Single-controller TeleOp OpMode with dynamic shooter target speed adjustment.
 * Maps driving, drive speed switching, intake, shooter flywheel, gate servo,
 * preset speed switching, and fine-tuning shooter target speed (+50 / -50 ticks/sec) onto Gamepad 1.
 */
@TeleOp
public class TeleOp_BaseCodeOneControllerSpeedAdjust extends OpMode {

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

        // Gamepad 1 Drivetrain Control
        handleDrivetrainDpadSpeedSwitching(gamepad1);
        driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Gamepad 1 Intake Control
        handleManualIntakeControl(gamepad1);

        // Gamepad 1 Shooter Flywheel & Target Speed Adjustment (+50 / -50 ticks/s)
        handleManualShootingControl(gamepad1);
        handleShooterSpeedControl(gamepad1);
        shooter.update();

        // ---------------- Telemetry Status Output ----------------
        telemetry.addData("Drive Speed Multiplier (G1 Dpad Right)", driveTrain.getSpeedMultiplier());
        telemetry.addLine("---------------- Statuses ---------------");
        telemetry.addData("Shooter Status (G1 RT/RB)", shooter.getShooterStatus());
        telemetry.addData("Intake Status (G1 LT/Back/LB)", intake.getIntakeStatus());
        telemetry.addData("Target Launch Speed (+50 G1 DpadUp / -50 G1 DpadDown)", "%.0f ticks/s", shooter.getTargetLaunchPower());
        telemetry.addData("Current Motor Speed", "%.0f ticks/s", shooter.getCurrentSpeed());
        telemetry.addData("Runtime (s)", "%.2f", TeleOpRuntime.seconds());
        telemetry.setMsTransmissionInterval(30);
        telemetry.update();
    }

    // ---------------------- Single Driver Input Handlers ----------------------

    private void handleManualShootingControl(Gamepad gamepad1) {
        if (gamepad1.right_trigger > 0.5) {
            shooter.start();
        } else if (gamepad1.right_bumper) {
            shooter.stop();
        }
    }

    private void handleShooterSpeedControl(Gamepad gamepad1) {
        // Cycle predefined speed presets on Dpad Left
        if (gamepad1.dpadLeftWasPressed()) {
            shooter.switchSpeed();
        }
        // Increase target speed by 50 ticks/sec on Dpad Up
        if (gamepad1.dpadUpWasPressed()) {
            shooter.increaseTargetSpeed(50);
        }
        // Decrease target speed by 50 ticks/sec on Dpad Down
        if (gamepad1.dpadDownWasPressed()) {
            shooter.decreaseTargetSpeed(50);
        }
    }

    private void handleManualIntakeControl(Gamepad gamepad1) {
        if (gamepad1.left_trigger > 0.5) {
            intake.start();
        } else if (gamepad1.back) {
            intake.out();
        } else if (gamepad1.left_bumper) {
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
