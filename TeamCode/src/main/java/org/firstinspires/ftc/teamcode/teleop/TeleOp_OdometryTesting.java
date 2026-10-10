package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.OdometryMovement;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * OpMode dedicated to testing goBILDA Pinpoint closed-loop odometry distance driving.
 * Displays real-time pose (X, Y, Heading), distance target settings, and error status on Driver Station telemetry.
 */
@TeleOp
public class TeleOp_OdometryTesting extends OpMode {

    // ---------------------- Subsystems & Hardware ----------------------
    public Shooter shooter = new Shooter();
    public Intake intake = new Intake();
    public MecanumDrive driveTrain = new MecanumDrive();
    public ElapsedTime TeleOpRuntime = new ElapsedTime();
    public Limelight limelight = new Limelight();
    public AprilTagAlignment aprilTagAlignment = new AprilTagAlignment();
    public OdometryMovement odometryMovement = new OdometryMovement();

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
        odometryMovement.init(hardwareMap, driveTrain);
    }

    /**
     * Resets OpMode runtime timer upon pressing Play.
     */
    @Override
    public void start() {
        TeleOpRuntime.reset();
    }

    /**
     * Main TeleOp frame loop executing drivetrain, odometry, and telemetry updates.
     */
    @Override
    public void loop() {

        // Handle Dpad Right speed switching and output manual joystick powers
        handleDrivetrainDpadSpeedSwitching(gamepad1);
        driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Update odometry computer pose and process manual distance driving inputs
        odometryMovement.update(gamepad1);

        // ---------------- Telemetry Output ----------------

        // Drivetrain Status
        telemetry.addLine("----------------- Drive Status -----------------");
        telemetry.addData("Drive Speed Multiplier [G1 DpadRight]", driveTrain.getSpeedMultiplier());

        // Real-time Odometry Position Pose
        telemetry.addLine("---------------- Odometry Pose ----------------");
        telemetry.addData("X Position (in)", "%.2f", odometryMovement.getHorizantalValue());
        telemetry.addData("Y Position (in)", "%.2f", odometryMovement.getVerticalValue());
        telemetry.addData("Heading (°)", "%.2f", odometryMovement.getHeadingDegrees());

        // Target Distance Controls & Movement Status
        telemetry.addLine("------------ Distance Driving Controls ------------");
        telemetry.addData("Distance to move (in) [G1 DpadUp/Down]", "%.1f", odometryMovement.getInputDistance());
        telemetry.addData("Direction [G1 A]", odometryMovement.getSelectedDirection());
        telemetry.addData("Step Size (in) [G1 B]", odometryMovement.getSelectedIncrement());
        telemetry.addData("Odometry Status [G1 RightTrigger]", odometryMovement.getStatus());
        telemetry.addData("Error", odometryMovement.getError());
        telemetry.addData("Target Distance", odometryMovement.getTargetDistance());

        telemetry.addData("Runtime (s)", "%.2f", TeleOpRuntime.seconds());

        telemetry.setMsTransmissionInterval(30);
        telemetry.update();
    }

    // ---------------------- Helper Input Handlers ----------------------

    private void handleManualShootingControl(Gamepad gamepad2) {
        if (gamepad2.right_trigger > 0.5) {
            shooter.start();
        } else if (gamepad2.right_bumper) {
            shooter.stop();
        }
    }

    private void handleShooterPresets(Gamepad gamepad2) {
        if (gamepad2.dpad_up) {
            shooter.switchSpeed();
        }
    }

    private void handleManualGateControl(Gamepad gamepad2) {
        if (gamepad2.x) {
            shooter.openGate();
        } else if (gamepad2.y) {
            shooter.closeGate();
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

    private void handleApriltagAlignmentIncrements(Gamepad gamepad1) {
        if (gamepad1.dpadUpWasPressed()) {
            aprilTagAlignment.addIndexValue();
        }
        if (gamepad1.dpadDownWasPressed()) {
            aprilTagAlignment.subIndexValue();
        }
        if (gamepad1.dpadLeftWasPressed()) {
            aprilTagAlignment.switchPIDModifyingValue();
        }
        if (gamepad1.bWasPressed()) {
            aprilTagAlignment.switchStepSize();
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
}
