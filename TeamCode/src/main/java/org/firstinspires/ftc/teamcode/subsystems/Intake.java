package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem controlling the intake roller motor and windmill mechanism.
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
     * Initializes intake and windmill motors from hardware map with brake behavior.
     */
    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "intake");
        windmill = hwMap.get(DcMotor.class, "windmill");
        windmill.setDirection(DcMotorSimple.Direction.FORWARD);
        windmill.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Runs intake and windmill inwards to pull game elements into the robot.
     */
    public void start() {
        intakeStatus = "Running";
        intake.setPower(INTAKE_IN);
        windmill.setPower(WINDMILL_IN);
    }

    /**
     * Runs intake and windmill in reverse to eject game elements.
     */
    public void out() {
        intakeStatus = "Reverse";
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
     * @return Status description for telemetry ("Running", "Reverse", "Off").
     */
    public String getIntakeStatus() {
        return intakeStatus;
    }
}
