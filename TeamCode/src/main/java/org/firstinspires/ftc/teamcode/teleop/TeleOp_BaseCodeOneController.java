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
 * Single-controller TeleOp OpMode for solo driver operation.
 * Maps driving, speed switching, intake, shooter flywheel, gate servo, and vision alignment onto Gamepad 1.
 */
@TeleOp
public class TeleOp_BaseCodeOneController extends OpMode {

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
     * Main TeleOp frame loop handling single-driver inputs, subsystem updates, and telemetry output.
     */
    @Override
    public void loop() {
        // Limelight vision & AprilTag alignment updates
        limelight.update();
        aprilTagAlignment.update();
        handleAutoRotation(gamepad1);
        // handleAutoStrafe(gamepad1);

        // Gamepad 1 Mecanum Drivetrain Control
        handleDrivtrainDpadSpeedSwitching(gamepad1);
        driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Gamepad 1 Intake Control
        handleManualIntakeControl(gamepad1);

        // Gamepad 1 Shooter Flywheel & Gate Control
        handleManualShootingControl(gamepad1);
        handleShooterSpeedControl();
        handleManualGateControl(gamepad1);
        shooter.update();

        // ---------------- Telemetry Status Output ----------------
        telemetry.addData("Robot Speed Multiplier G1 Dpad Right", driveTrain.getSpeedMultiplier());

        telemetry.addData("Shooter Status G1 RT/RB", shooter.getShooterStatus());
        telemetry.addData("Gate Status G1 X/Y", shooter.getGateStatus());
        telemetry.addData("Intake Status G1 LT/Back/LB", intake.getIntakeStatus());
        telemetry.addData("Target Launch Power G1 Dpad Up", shooter.getTargetLaunchPower());
        telemetry.addData("Current Motor Speed", shooter.getCurrentSpeed());

        telemetry.addData("Runtime:", TeleOpRuntime.seconds());
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

    private void handleShooterSpeedControl() {
        if (gamepad1.dpadUpWasPressed()) {
            shooter.switchSpeed();
        }
    }

    private void handleManualGateControl(Gamepad gamepad1) {
        if (gamepad1.x) {
            shooter.openGate();
        } else if (gamepad1.y) {
            shooter.closeGate();
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
}
