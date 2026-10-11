package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem controlling the intake roller motor and optional windmill mechanism.
 * Handles intaking game elements, reversing (spitting out), and stopping.
 */
public class Intake {

    // ---------------------- Hardware Motors ----------------------
    private DcMotor intake;
    private DcMotor windmill;

    // ---------------------- Default Motor Power Constants ----------------------
    public static final double INTAKE_IN = 0.7;
    public static final double INTAKE_OUT = -0.7;
    public static final double WINDMILL_IN = -1.0;
    public static final double WINDMILL_OUT = 1.0;

    // ---------------------- Status Variable ----------------------
    String intakeStatus = "Off";

    /**
     * Initializes intake motor and optional windmill motor from hardware map with brake behavior.
     * Safely catches missing "windmill" configuration if not present.
     */
    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        try {
            windmill = hwMap.get(DcMotor.class, "windmill");
            if (windmill != null) {
                windmill.setDirection(DcMotorSimple.Direction.FORWARD);
                windmill.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } catch (Exception e) {
            windmill = null; // Windmill hardware motor not configured/installed
        }
    }

    /**
     * Runs intake and optional windmill inwards to pull game elements into the robot.
     */
    public void start() {
        intakeStatus = "Running";
        intake.setPower(INTAKE_IN);
        if (windmill != null) {
            windmill.setPower(WINDMILL_IN);
        }
    }

    /**
     * Runs intake and optional windmill in reverse to eject game elements.
     */
    public void out() {
        intakeStatus = "Reverse";
        intake.setPower(INTAKE_OUT);
        if (windmill != null) {
            windmill.setPower(WINDMILL_OUT);
        }
    }

    /**
     * Stops intake and optional windmill motors completely.
     */
    public void stop() {
        intakeStatus = "Off";
        intake.setPower(0);
        if (windmill != null) {
            windmill.setPower(0);
        }
    }

    /**
     * Sets custom speed for intake and optional windmill.
     */
    public void switchSpeed(double speed) {
        intake.setPower(speed);
        if (windmill != null) {
            windmill.setPower(speed);
        }
    }

    /**
     * @return Status description for telemetry ("Running", "Reverse", "Off").
     */
    public String getIntakeStatus() {
        return intakeStatus;
    }
}
