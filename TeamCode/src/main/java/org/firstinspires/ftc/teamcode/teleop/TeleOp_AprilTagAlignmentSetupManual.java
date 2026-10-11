package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * OpMode providing manual live tuning for AprilTag PID parameters (kP_rotation, kD_rotation).
 * Allows drivers to adjust gains with Dpad Up/Down, switch active parameter with Dpad Left, and change step size with B button.
 */
@TeleOp
public class TeleOp_AprilTagAlignmentSetupManual extends OpMode {

    // ---------------------- Subsystem Hardware Objects ----------------------
    public Shooter shooter = new Shooter();
    public Intake intake = new Intake();
    public MecanumDrive driveTrain = new MecanumDrive();
    public ElapsedTime TeleOpRuntime = new ElapsedTime();
    public Limelight limelight = new Limelight();
    public AprilTagAlignment aprilTagAlignment = new AprilTagAlignment();

    /**
     * Initializes Limelight vision camera and robot subsystems.
     */
    @Override
    public void init() {
        Log.v(Limelight.TAG, "Initializing limelight");
        limelight.init(hardwareMap);
        aprilTagAlignment.init(limelight, TeleOpRuntime);
        driveTrain.init(hardwareMap, aprilTagAlignment);
        intake.init(hardwareMap);
        shooter.init(hardwareMap, intake);
    }

    /**
     * Starts vision camera feed and resets runtime timers upon pressing Play.
     */
    @Override
    public void start() {
        TeleOpRuntime.reset();
        limelight.start();
        aprilTagAlignment.start();
    }

    /**
     * Main TeleOp frame loop handling manual driving, vision alignment, PID tuning, and telemetry.
     */
    @Override
    public void loop() {
        // Limelight vision update
        limelight.update();

        // AprilTag Alignment update and manual driver controls
        aprilTagAlignment.update();
        handleAutoRotation(gamepad1);
        handleApriltagAlignmentIncrements(gamepad1);
        // handleAutoStrafe(gamepad1);

        // Drivetrain speed switching and manual driving
        handleDrivetrainDpadSpeedSwitching(gamepad1);
        driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Intake controls (Gamepad 2)
        handleManualIntakeControl(gamepad2);

        // Shooter and Gate controls (Gamepad 2)
        handleManualShootingControl(gamepad2);
        handleShooterSpeedControl(gamepad2);
        handleManualGateControl(gamepad2);
        shooter.update();

        // ---------------- Telemetry Output ----------------
        telemetry.addData("Robot Speed Multiplier(G1DpadRight)", driveTrain.getSpeedMultiplier());

        telemetry.addData("Target Launch Power(G2DpadUpRightDown)", shooter.getTargetLaunchPower());
        telemetry.addData("Current Motor Speed", shooter.getCurrentSpeed());
        telemetry.addData("Shooter Status(G2RightTrigger/Bumper)", shooter.getShooterStatus());
        telemetry.addData("Gate Status(G2X/Y)", shooter.getGateStatus());
        telemetry.addData("Intake Status(G2LeftTrigger/Bumper/Back)", intake.getIntakeStatus());

        telemetry.addLine("----------------April Tags ---------------");
        telemetry.addData("kP_Rotation(G1DpadUp/Down)", aprilTagAlignment.getkP_rotation());
        telemetry.addData("kD_Rotation(G1DpadUp/Down)", aprilTagAlignment.getkD_rotation());
        // telemetry.addData("kP_Strafe(G1DpadUp/Down)", aprilTagAlignment.getkP_strafe());
        // telemetry.addData("kD_Strafe(G1DpadUp/Down)", aprilTagAlignment.getkD_strafe());
        telemetry.addData("Step Size(G1B)", aprilTagAlignment.getStepSize());
        telemetry.addData("Currently Modifying(G1DpadLeft)", aprilTagAlignment.getCurrentlyModifying());
        telemetry.addData("HorAngleDelta", limelight.getHorizontalDelta());
        telemetry.addData("VerAngleDelta", limelight.getVerticalDelta());
        telemetry.addData("IsValid", limelight.isTargetVisible());

        Pose3D botPose = limelight.getBotPose();
        if (botPose != null && limelight.isTargetVisible()) {
            telemetry.addData("3D X (m)", "%.2f", botPose.getPosition().x);
            telemetry.addData("3D Y (m)", "%.2f", botPose.getPosition().y);
            telemetry.addData("3D Z (m)", "%.2f", botPose.getPosition().z);
            telemetry.addData("Yaw (°)", "%.2f", botPose.getOrientation().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
            telemetry.addData("Pitch (°)", "%.2f", botPose.getOrientation().getPitch(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
            telemetry.addData("Roll (°)", "%.2f", botPose.getOrientation().getRoll(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
        } else {
            telemetry.addData("3D BotPose", "No 3D Pose Available");
        }

        telemetry.addData("Runtime:", TeleOpRuntime.seconds());
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

    private void handleShooterSpeedControl(Gamepad gamepad2) {
        if (gamepad2.dpadUpWasPressed()) {
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
