package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem controlling the intake roller motor and the windmill feeder.
 *
 * The roller pulls game elements into the robot. The windmill moves them into the shooter.
 * start() runs only the roller; stop() stops the roller and the windmill. The Shooter
 * runs the roller and windmill once the flywheel is at speed, and stops only the windmill
 * when it turns off.
 */
public class Intake {

    // ---------------------- Hardware Motors ----------------------
    private DcMotor intake;
    private DcMotor windmill;

    // ---------------------- Default Motor Power Constants ----------------------
    // Intake motor direction is set to REVERSE in init(), and "in" uses negative power
    // on top of that. Verify on the robot which way the roller physically spins.
    public static final double INTAKE_POWER = 0.8;
    public static final double INTAKE_IN = -INTAKE_POWER;
    public static final double INTAKE_OUT = INTAKE_POWER;
    public static final double WINDMILL_POWER = 1.0;
    public static final double WINDMILL_IN = WINDMILL_POWER;
    public static final double WINDMILL_OUT = -WINDMILL_POWER;

    // ---------------------- Status Variable ----------------------
    private String intakeStatus = "Off";

    /**
     * Initializes intake roller and windmill motors with brake behavior.
     */
    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        windmill = hwMap.get(DcMotor.class, "windmill");
        windmill.setDirection(DcMotorSimple.Direction.FORWARD);
        windmill.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        windmill.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Runs the roller inwards at default power to pull game elements into the robot.
     */
    public void start() {
        intakeStatus = "Running Inwards";
        intake.setPower(INTAKE_IN);
    }

    /**
     * Runs the roller in reverse (and the windmill out) to eject game elements.
     */
    public void out() {
        intakeStatus = "Running Reverse";
        intake.setPower(INTAKE_OUT);
        windmill.setPower(WINDMILL_OUT);
    }

    /**
     * Stops the roller and the windmill.
     */
    public void stop() {
        stopRoller();
        stopWindmill();
    }

    /**
     * Stops only the roller, leaving the windmill as it is.
     */
    public void stopRoller() {
        intakeStatus = "Off";
        intake.setPower(0);
    }

    /**
     * Sets a custom roller speed. Positive = inwards, negative = reverse (ejects),
     * matching the status text. Range is clamped to [-1, 1].
     */
    public void switchSpeed(double speed) {
        speed = Math.max(-1.0, Math.min(1.0, speed));

        if (speed > 0) {
            intakeStatus = String.format(java.util.Locale.US, "Running Inwards at (%.2f)", speed);
            intake.setPower(-speed); // inward is negative motor power
        } else if (speed < 0) {
            intakeStatus = String.format(java.util.Locale.US, "Running Reverse at (%.2f)", speed);
            intake.setPower(-speed); // outward is positive motor power
            windmill.setPower(WINDMILL_OUT);
        } else {
            intakeStatus = "Off";
            intake.setPower(0);
            windmill.setPower(0);
        }
    }

    /**
     * Starts the windmill inwards to feed elements into the shooter.
     */
    public void startWindmill() {
        windmill.setPower(WINDMILL_IN);
    }

    /**
     * Stops the windmill.
     */
    public void stopWindmill() {
        windmill.setPower(0);
    }

    /**
     * @return Status description for telemetry ("Running Inwards", "Running Reverse", "Off").
     */
    public String getIntakeStatus() {
        return intakeStatus;
    }
}