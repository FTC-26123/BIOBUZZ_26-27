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
 * Standard dual-controller TeleOp OpMode for competition driving.
 * Driver 1 controls Mecanum driving, speed multiplier switching, and AprilTag auto-rotation.
 * Driver 2 controls intake roller/windmill, shooter flywheel speeds, and servo gate release.
 */
@TeleOp
public class TeleOp_BaseCode extends OpMode {

    // ---------------------- Subsystems & Hardware Objects ----------------------
    public Shooter shooter = new Shooter();
    public Intake intake = new Intake();
    public MecanumDrive driveTrain = new MecanumDrive();
    public ElapsedTime TeleOpRuntime = new ElapsedTime();
    public Limelight limelight = new Limelight();
    public AprilTagAlignment aprilTagAlignment = new AprilTagAlignment();

    /**
     * Initializes hardware mapping and subsystem dependencies.
     */
    @Override
    public void init() {
        limelight.init(hardwareMap);
        aprilTagAlignment.init(limelight, TeleOpRuntime);
        driveTrain.init(hardwareMap, aprilTagAlignment);
        intake.init(hardwareMap);
        shooter.init(hardwareMap, intake);
    }

    /**
     * Resets runtime timer and starts Limelight vision processing upon pressing Play.
     */
    @Override
    public void start() {
        TeleOpRuntime.reset();
        limelight.start();
    }

    /**
     * Main TeleOp frame loop handling driver controls, subsystem updates, and status telemetry.
     */
    @Override
    public void loop() {
        // Limelight vision & AprilTag alignment updates
        limelight.update();
        aprilTagAlignment.update();
        handleAutoRotation(gamepad1);
        // handleAutoStrafe(gamepad1);

        // Driver 1 Drivetrain Control
        handleDrivtrainDpadSpeedSwitching(gamepad1);
        driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Driver 2 Intake Control
        handleManualIntakeControl(gamepad2);

        // Driver 2 Shooter Flywheel & Gate Control
        handleManualShootingControl(gamepad2);
        handleShooterSpeedControl(gamepad2);
        handleManualGateControl(gamepad2);
        shooter.update();

        // ---------------- Telemetry Status Output ----------------
        telemetry.addData("Robot Speed Multiplier G1 Dpad Right", driveTrain.getSpeedMultiplier());

        telemetry.addLine("---------------- Statuses ---------------");
        telemetry.addData("Shooter Status G2 RT", shooter.getShooterStatus());
        telemetry.addData("Gate Status G2 X/Y", shooter.getGateStatus());
        telemetry.addData("Intake Status G2 LT/Back/LB", intake.getIntakeStatus());
        telemetry.addData("Target Launch Power G2 Dpad Up", shooter.getTargetLaunchPower());
        telemetry.addData("Current Motor Speed", shooter.getCurrentSpeed());

        telemetry.addData("Runtime:", TeleOpRuntime.seconds());
        telemetry.setMsTransmissionInterval(30);
        telemetry.update();
    }

    // ---------------------- Driver Input Handlers ----------------------

    private void handleManualShootingControl(Gamepad gamepad2) {
        if (gamepad2.right_bumper) {
            shooter.stop();
        } else {
            if (gamepad2.right_trigger > 0.5) shooter.start();
            shooter.setFireRequested(gamepad2.right_trigger > 0.5);
        }
    }

    private void handleShooterSpeedControl(Gamepad gamepad2) {
        if (gamepad2.dpadUpWasPressed()) {
            shooter.switchSpeed();
        }
    }

    private void handleManualGateControl(Gamepad gamepad2) {
        if (gamepad2.y || gamepad2.right_bumper) {
            shooter.closeGate();
        } else if (gamepad2.x) {
            shooter.openGate();
        }
    }

    private void handleManualIntakeControl(Gamepad gamepad2) {
        if (gamepad2.left_bumper) {
            intake.stop();
        } else if (gamepad2.back) {
            intake.out();
        } else if (gamepad2.left_trigger > 0.5) {
            intake.start();
        }
    }

    private void handleDrivtrainDpadSpeedSwitching(Gamepad gamepad1) {
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

    private void handleAutoStrafe(Gamepad gamepad1) {
        if (gamepad1.left_bumper) {
            aprilTagAlignment.autoStrafe();
        } else {
            aprilTagAlignment.setManualStrafing();
        }
    }

    @Override
    public void stop() {
        shooter.stop();
        driveTrain.stop();
        limelight.stop();
    }
}
