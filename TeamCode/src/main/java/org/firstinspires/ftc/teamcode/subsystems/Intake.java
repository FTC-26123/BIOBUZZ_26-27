package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem controlling the intake roller motor, windmill feeder, and transfer motor.
 *
 * <p>The transfer is a DC motor named {@code "transfer"} in the robot configuration (it replaced the old
 * continuous servo). It brakes at zero power so it does not coast on after being stopped. Every stop path in
 * this class goes through {@link #stopTransfer()}.
 */
public class Intake {

    // ---------------------- Hardware Motors & Servos ----------------------
    private DcMotor intake;
    private DcMotor windmill;
    private DcMotor transfer;

    // ---------------------- Default Motor & Servo Power Constants ----------------------
    public static final double INTAKE_POWER = 0.80;
    public static final double INTAKE_IN = -INTAKE_POWER;
    public static final double INTAKE_OUT = INTAKE_POWER;
    public static final double WINDMILL_POWER = 1.0;
    public static final double WINDMILL_IN = WINDMILL_POWER;
    public static final double WINDMILL_OUT = -WINDMILL_POWER;
    /** Signed power that moves elements INWARD (toward the shooter), same sign convention the servo used. */
    public static final double TRANSFER_POWER = -1.0;
    /** Flip this one constant if the transfer motor spins the wrong way on the robot. */
    public static final DcMotorSimple.Direction TRANSFER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    /** Kept so older references still compile; prefer {@link #TRANSFER_POWER}. */
    public static final double TRANSFER_SERVO_POWER = TRANSFER_POWER;

    // ---------------------- Status Variable ----------------------
    private String intakeStatus = "Off";

    /**
     * Initializes the intake roller, windmill and transfer motors from the hardware map. The transfer starts
     * stopped with BRAKE zero-power behavior.
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

        transfer = hwMap.get(DcMotor.class, "transfer");
        transfer.setDirection(TRANSFER_DIRECTION);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Brake at zero power so the transfer does not coast after a stop.
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopTransfer();
    }

    /**
     * Runs the roller and transfer motor inwards at default power to pull game elements into the robot.
     */
    public void start() {
        intakeStatus = "Running Inwards";
        intake.setPower(INTAKE_IN);
        transfer.setPower(TRANSFER_POWER);
    }

    /**
     * Runs the roller, windmill, and transfer motor in reverse to eject game elements.
     */
    public void out() {
        intakeStatus = "Running Reverse";
        intake.setPower(INTAKE_OUT);
        windmill.setPower(WINDMILL_OUT);
        transfer.setPower(-TRANSFER_POWER);
    }

    /**
     * Stops the roller, windmill, and transfer motor.
     */
    public void stop() {
        stopRoller();
        stopWindmill();
        stopTransfer();
    }

    /**
     * Stops only the roller, leaving the windmill and transfer motor as they are.
     */
    public void stopRoller() {
        intakeStatus = "Off";
        intake.setPower(0);
    }

    /**
     * Sets a custom roller speed. Positive = inwards, negative = reverse (ejects).
     */
    public void switchSpeed(double speedInput) {
        double speed = Math.max(-1.0, Math.min(1.0, speedInput));

        if (speed > 0) {
            intakeStatus = String.format(java.util.Locale.US, "Running Inwards at (%.2f)", speed);
            intake.setPower(-speed);
            transfer.setPower(TRANSFER_POWER);
        } else if (speed < 0) {
            intakeStatus = String.format(java.util.Locale.US, "Running Reverse at (%.2f)", speed);
            intake.setPower(-speed);
            windmill.setPower(WINDMILL_OUT);
            transfer.setPower(-TRANSFER_POWER);
        } else {
            intakeStatus = "Off";
            intake.setPower(0);
            stopWindmill();
            stopTransfer();
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
     * Starts the transfer motor inwards to feed elements into the shooter.
     */
    public void startTransfer() {
        transfer.setPower(TRANSFER_POWER);
    }

    /**
     * Stops the transfer motor and re-asserts BRAKE so it cannot be left coasting.
     */
    public void stopTransfer() {
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setPower(0.0);
    }

    /** @deprecated the transfer is a motor now; use {@link #startTransfer()}. */
    @Deprecated
    public void startTransferServo() { startTransfer(); }

    /** @deprecated the transfer is a motor now; use {@link #stopTransfer()}. */
    @Deprecated
    public void stopTransferServo() { stopTransfer(); }

    /**
     * @return Status description for telemetry ("Running Inwards", "Running Reverse", "Off").
     */
    public String getIntakeStatus() {
        return intakeStatus;
    }
}
