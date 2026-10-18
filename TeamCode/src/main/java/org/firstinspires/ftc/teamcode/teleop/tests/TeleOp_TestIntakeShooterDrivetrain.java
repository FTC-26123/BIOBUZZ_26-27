package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Test OpMode to test Intake and Shooter subsystems together.
 * Supports drivetrain driving, manual intake controls, shooter speed switching, and gate servo operations.
 */
@TeleOp
public class TeleOp_TestIntakeShooterDrivetrain extends OpMode {

    // ---------------------- Subsystem Objects ----------------------
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

        // Driver 2: Custom Intake Power Adjustments & Shooter Speed Preset
        handleCustomIntakePowerAdjustments(gamepad2);
        handleShooterSpeedControl(gamepad2);

        // Driver 2: Manual Intake & Shooter Controls
        handleManualIntakeControl(gamepad2);
        handleManualShootingControl(gamepad2);
        handleManualGateControl(gamepad2);

        // Shooter subsystem loop update
        shooter.update();

        // ---------------- Telemetry Output ----------------
        telemetry.addLine("================ DRIVE STATUS ================");
        telemetry.addData("Drive Speed Multiplier [G1 DpadRight]", driveTrain.getSpeedMultiplier());

        telemetry.addLine("================ INTAKE STATUS ================");
        telemetry.addData("Intake Status [G2 LT/LB/Back]", intake.getIntakeStatus());
        telemetry.addData("Custom Intake Power [G2 DpadDown/Left]", "%.2f", customIntakePower);
        telemetry.addData("Intake Adjustment Step Size [G2 B]", intakeStepSizes[intakeStepIndex]);

        telemetry.addLine("================ SHOOTER STATUS ================");
        telemetry.addData("Shooter Status [G2 RT/RB]", shooter.getShooterStatus());
        telemetry.addData("Gate Status [G2 X/Y]", shooter.getGateStatus());
        telemetry.addData("Target Launch Velocity [G2 DpadUp]", "%.0f ticks/s", shooter.getTargetLaunchPower());
        telemetry.addData("Current Motor Velocity", "%.0f ticks/s", shooter.getCurrentSpeed());

        telemetry.addData("Runtime (s)", "%.2f", runtime.seconds());
        telemetry.setMsTransmissionInterval(30);
        telemetry.update();
    }

    /**
     * Controls shooter start on Right Trigger (> 0.5) and stop on Right Bumper.
     */
    private void handleManualShootingControl(Gamepad gamepad2) {
        if (gamepad2.right_bumper) {
            shooter.stop();
        } else {
            if (gamepad2.right_trigger > 0.5) shooter.start();
            shooter.setFireRequested(gamepad2.right_trigger > 0.5);
        }
    }

    /**
     * Cycles shooter target launch velocity preset (2000 -> 1500 -> 1000 ticks/s) on Dpad Up.
     */
    private void handleShooterSpeedControl(Gamepad gamepad2) {
        if (gamepad2.dpadUpWasPressed()) {
            shooter.switchSpeed();
        }
    }

    /**
     * Opens launcher gate on Gamepad 2 X and closes gate on Gamepad 2 Y.
     */
    private void handleManualGateControl(Gamepad gamepad2) {
        if (gamepad2.y || gamepad2.right_bumper) {
            shooter.closeGate();
        } else if (gamepad2.x) {
            shooter.openGate();
        }
    }

    /**
     * Controls intake start on Left Trigger (> 0.5), reverse on Back button, and stop on Left Bumper.
     */
    private void handleManualIntakeControl(Gamepad gamepad2) {
        if (gamepad2.left_bumper) {
            intake.stop();
        } else if (gamepad2.back) {
            intake.out();
        } else if (gamepad2.left_trigger > 0.5) {
            intake.switchSpeed(customIntakePower);
        }
    }

    /**
     * Adjusts custom intake power using Gamepad 2 Dpad Down / Left and cycles step size on B button.
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
     * Switches drivetrain speed multiplier on Gamepad 1 Dpad Right.
     */
    private void handleDrivetrainDpadSpeedSwitching(Gamepad gamepad1) {
        if (gamepad1.dpadRightWasPressed()) {
            driveTrain.switchSpeed();
        }
    }

    @Override
    public void stop() {
        shooter.stop();
        driveTrain.stop();
    }
}
