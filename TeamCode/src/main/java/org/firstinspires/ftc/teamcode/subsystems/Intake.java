package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem controlling the intake roller motor, windmill feeder, and transfer CRServo.
 */
public class Intake {

    // ---------------------- Hardware Motors & Servos ----------------------
    private DcMotor intake;
    private DcMotor windmill;
    private CRServo transferServo;

    // ---------------------- Default Motor & Servo Power Constants ----------------------
    public static final double INTAKE_POWER = 0.8;
    public static final double INTAKE_IN = -INTAKE_POWER;
    public static final double INTAKE_OUT = INTAKE_POWER;
    public static final double WINDMILL_POWER = 1.0;
    public static final double WINDMILL_IN = WINDMILL_POWER;
    public static final double WINDMILL_OUT = -WINDMILL_POWER;
    public static final double TRANSFER_SERVO_POWER = -1.0;

    // ---------------------- Status Variable ----------------------
    private String intakeStatus = "Off";

    /**
     * Initializes intake roller motor, windmill motor, and transfer CRServo from hardware map.
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

        transferServo = hwMap.get(CRServo.class, "transferServo");
    }

    /**
     * Runs the roller and transfer servo inwards at default power to pull game elements into the robot.
     */
    public void start() {
        intakeStatus = "Running Inwards";
        intake.setPower(INTAKE_IN);
        transferServo.setPower(TRANSFER_SERVO_POWER);
    }

    /**
     * Runs the roller, windmill, and transfer servo in reverse to eject game elements.
     */
    public void out() {
        intakeStatus = "Running Reverse";
        intake.setPower(INTAKE_OUT);
        windmill.setPower(WINDMILL_OUT);
        transferServo.setPower(-TRANSFER_SERVO_POWER);
    }

    /**
     * Stops the roller, windmill, and transfer servo.
     */
    public void stop() {
        stopRoller();
        stopWindmill();
        stopTransferServo();
    }

    /**
     * Stops only the roller, leaving the windmill and transfer servo as they are.
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
            transferServo.setPower(TRANSFER_SERVO_POWER);
        } else if (speed < 0) {
            intakeStatus = String.format(java.util.Locale.US, "Running Reverse at (%.2f)", speed);
            intake.setPower(-speed);
            windmill.setPower(WINDMILL_OUT);
            transferServo.setPower(-TRANSFER_SERVO_POWER);
        } else {
            intakeStatus = "Off";
            intake.setPower(0);
            stopWindmill();
            stopTransferServo();
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
     * Starts the transfer CRServo inwards to feed elements into the shooter.
     */
    public void startTransferServo() {
        transferServo.setPower(TRANSFER_SERVO_POWER);
    }

    /**
     * Stops the transfer CRServo.
     */
    public void stopTransferServo() {
        transferServo.setPower(0.0);
    }

    /**
     * @return Status description for telemetry ("Running Inwards", "Running Reverse", "Off").
     */
    public String getIntakeStatus() {
        return intakeStatus;
    }
}
