package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Diagnostic testing OpMode for Intake and Shooter subsystems.
 * Allows testing custom intake powers and shooter preset launch velocities.
 */
@TeleOp
public class TeleOp_TestIntakeShooter extends OpMode {

    // ---------------------- Subsystems & Hardware ----------------------
    public Shooter shooter = new Shooter();
    public Intake intake = new Intake();
    public MecanumDrive driveTrain = new MecanumDrive();
    public ElapsedTime runtime = new ElapsedTime();

    // ---------------------- Custom Intake Power Settings ----------------------
    private double customIntakePower = 0.70;
    private final double[] intakeStepSizes = {0.05, 0.10, 0.25};
    private int intakeStepIndex = 1;

    @Override
    public void init() {
        driveTrain.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap, intake);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Driver 1: Mecanum Drivetrain Control
        handleDrivetrainDpadSpeedSwitching(gamepad1);
        driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Driver 2: Custom Intake Power Adjustments & Shooter Preset Switching
        handleCustomIntakePowerAdjustments(gamepad2);
        handleShooterPresetSwitching(gamepad2);

        // Driver 2: Subsystem Operations
        handleManualIntakeControl(gamepad2);
        handleManualShooterControl(gamepad2);

        shooter.update();

        // ---------------- Telemetry Output ----------------
        telemetry.addLine("================ TEST CONTROLS ================");
        telemetry.addData("Custom Intake Target Power [G2 DpadUp/Down]", "%.2f", customIntakePower);
        telemetry.addData("Intake Adjustment Step Size [G2 B]", intakeStepSizes[intakeStepIndex]);
        telemetry.addData("Shooter Preset Target Power [G2 DpadUp]", "%.0f ticks/s", shooter.getTargetLaunchPower());

        telemetry.addLine("================ SUBSYSTEM STATUS ================");
        telemetry.addData("Intake Status [G2 LT/LB/Back]", intake.getIntakeStatus());
        telemetry.addData("Shooter Status [G2 RT/RB]", shooter.getShooterStatus());
        telemetry.addData("Shooter Current Speed", "%.0f ticks/s", shooter.getCurrentSpeed());
        telemetry.addData("Gate Status [G2 X/Y]", shooter.getGateStatus());
        telemetry.addData("Drive Speed Multiplier [G1 DpadRight]", driveTrain.getSpeedMultiplier());

        telemetry.setMsTransmissionInterval(30);
        telemetry.update();
    }

    /**
     * Adjusts custom intake power using Gamepad 2 Dpad Up / Down and cycles step size on B button.
     */
    private void handleCustomIntakePowerAdjustments(Gamepad gamepad2) {
        if (gamepad2.bWasPressed()) {
            intakeStepIndex = (intakeStepIndex + 1) % intakeStepSizes.length;
        }

        double step = intakeStepSizes[intakeStepIndex];
        if (gamepad2.dpadDownWasPressed()) {
            customIntakePower = Math.max(0.0, customIntakePower - step);
        } else if (gamepad2.dpadLeftWasPressed()) {
            customIntakePower = Math.min(1.0, customIntakePower + step);
        }
    }

    /**
     * Cycles shooter preset speeds (2000 -> 1500 -> 1000 ticks/s) on Gamepad 2 Dpad Up.
     */
    private void handleShooterPresetSwitching(Gamepad gamepad2) {
        if (gamepad2.dpadUpWasPressed()) {
            shooter.switchSpeed();
        }
    }

    /**
     * Runs intake at custom power on Left Trigger, reverse on Back, or stops on Left Bumper.
     */
    private void handleManualIntakeControl(Gamepad gamepad2) {
        if (gamepad2.left_trigger > 0.3) {
            intake.switchSpeed(customIntakePower);
        } else if (gamepad2.back) {
            intake.switchSpeed(-customIntakePower);
        } else if (gamepad2.left_bumper) {
            intake.stop();
        }
    }

    /**
     * Spools shooter flywheel to preset velocity on Right Trigger, or stops on Right Bumper.
     */
    private void handleManualShooterControl(Gamepad gamepad2) {
        if (gamepad2.right_trigger > 0.3) {
            shooter.start();
        } else if (gamepad2.right_bumper) {
            shooter.stop();
        }
    }
    private void handleDrivetrainDpadSpeedSwitching(Gamepad gamepad1) {
        if (gamepad1.dpadRightWasPressed()) {
            driveTrain.switchSpeed();
        }
    }
}
