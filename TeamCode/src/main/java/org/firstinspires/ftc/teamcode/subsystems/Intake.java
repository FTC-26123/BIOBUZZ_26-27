package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem controlling the intake roller motor and required windmill mechanism.
 * Handles intaking game elements, reversing (spitting out), custom power adjustments, and stopping.
 */
public class Intake {

    // ---------------------- Hardware Motors ----------------------
    private DcMotor intake;
    private DcMotor windmill;

    // ---------------------- Default Motor Power Constants ----------------------
    public static final double INTAKE_POWER = 0.8;
    public static final double INTAKE_IN = -INTAKE_POWER;
    public static final double INTAKE_OUT = INTAKE_POWER;
    public static final double WINDMILL_POWER = 1.0;
    public static final double WINDMILL_IN = WINDMILL_POWER;
    public static final double WINDMILL_OUT = -WINDMILL_POWER;

    // ---------------------- Status Variable ----------------------
    String intakeStatus = "Off";

    /**
     * Initializes intake motor and required windmill motor from hardware map with brake behavior.
     */
    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        windmill = hwMap.get(DcMotor.class, "windmill");
        windmill.setDirection(DcMotorSimple.Direction.FORWARD);
        windmill.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Runs intake and windmill inwards at default power to pull game elements into the robot.
     */
    public void start() {
        intakeStatus = "Running Inwards";
        intake.setPower(INTAKE_IN);
        windmill.setPower(WINDMILL_IN);
    }

    /**
     * Runs intake and windmill in reverse at default power to eject game elements.
     */
    public void out() {
        intakeStatus = "Running Reverse";
        intake.setPower(INTAKE_OUT);
        windmill.setPower(WINDMILL_OUT);
    }

    /**
     * Stops intake and windmill motors completely.
     */
    public void stop() {
        intakeStatus = "Off";
        intake.setPower(0);
        windmill.setPower(0);
    }

    /**
     * Sets custom speed/power for intake and runs windmill at max power in matching direction.
     */
    public void switchSpeed(double speed) {
        if (speed > 0) {
            intakeStatus = String.format(java.util.Locale.US, "Running Inwards at (%.2f)", speed);
            windmill.setPower(WINDMILL_IN);
        } else if (speed < 0) {
            intakeStatus = String.format(java.util.Locale.US, "Running Reverse at (%.2f)", speed);
            windmill.setPower(WINDMILL_OUT);
        } else {
            intakeStatus = "Off";
            windmill.setPower(0);
        }

        intake.setPower(speed);
    }

    /**
     * @return Status description for telemetry ("Running Inwards", "Running Reverse", "Off").
     */
    public String getIntakeStatus() {
        return intakeStatus;
    }
}
