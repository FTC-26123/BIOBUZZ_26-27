package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem responsible for controlling a 4-motor Mecanum Drivetrain.
 * Supports manual joystick driving, driver speed multiplier switching, and automated offsets
 * injected by vision alignment (AprilTagAlignment) and odometry positioning (OdometryMovement).
 */
public class MecanumDrive {

    // ---------------------- Hardware Motors & Subsystem References ----------------------
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private AprilTagAlignment aprilTagAlignment;
    private OdometryMovement odometryMovement;
    private TeleOpPositionHold positionHold;
    private String holdUnavailable = "Disabled";

    /** Opt in only from standard TeleOps; diagnostics and autonomous retain their own ownership. */
    public void enablePositionHold(HardwareMap hwMap) {
        try {
            positionHold = TeleOpPositionHold.create(hwMap);
        } catch (RuntimeException e) {
            positionHold = null;
            holdUnavailable = "Unavailable - check Pinpoint configuration";
            stop();
        }
    }

    public void driveWithPositionHold(double y, double x, double rx, boolean bypass) {
        // Per-axis dead zone: a resting or drifting stick can never turn or creep the robot.
        y = TeleOpPositionHold.deadband(y); x = TeleOpPositionHold.deadband(x); rx = TeleOpPositionHold.deadband(rx);
        if (positionHold != null && positionHold.update(y, x, rx, bypass, System.nanoTime() / 1e9)) return;
        if (!bypass && TeleOpPositionHold.sticksIdle(y, x, rx)) {
            y = 0; x = 0; rx = 0;
        }
        drive(y, x, rx);
    }

    /**
     * Same as {@link #driveWithPositionHold(double, double, double, boolean)}, plus AIM. While aiming with the
     * left stick still, the robot's spot is locked (pushes are resisted) and the hold turns it to the camera's
     * aim point. Moving the left stick drives normally, with the camera doing the turning.
     *
     * @param bypass hold off completely (right stick button)
     * @param aiming AIM button held
     */
    public void driveWithPositionHold(double y, double x, double rx, boolean bypass, boolean aiming) {
        y = TeleOpPositionHold.deadband(y); x = TeleOpPositionHold.deadband(x); rx = TeleOpPositionHold.deadband(rx);
        if (aiming && !bypass && positionHold != null && aprilTagAlignment != null
                && aprilTagAlignment.hasTargetTracker()) {
            boolean target = !Double.isNaN(aprilTagAlignment.getAimTurnDeg());
            if (!target && !TeleOpPositionHold.sticksIdle(0, 0, rx)) {
                // No target in view and the driver is turning by hand: let them.
                driveWithPositionHold(y, x, rx, true);
                return;
            }
            // Same PD turn power as when driving (Aim kP / kD / max power), with the spot locked.
            double turnPower = target ? aprilTagAlignment.getAddedRotationPower() : Double.NaN;
            if (positionHold.updateAiming(y, x, turnPower, System.nanoTime() / 1e9)) return;
            drive(y, x, rx);
            return;
        }
        driveWithPositionHold(y, x, rx, bypass || aiming);
    }

    /** The TeleOp position hold, or null if it is not enabled / not available. */
    public TeleOpPositionHold getPositionHold() { return positionHold; }

    public String getPositionHoldStatus() {
        return positionHold == null ? holdUnavailable : positionHold.getStatus();
    }

    public void stop() {
        if (positionHold != null) positionHold.release();
        // Direct zero bypasses alignment offsets and cached Pedro motor commands.
        if (frontLeftMotor != null) frontLeftMotor.setPower(0);
        if (frontRightMotor != null) frontRightMotor.setPower(0);
        if (backLeftMotor != null) backLeftMotor.setPower(0);
        if (backRightMotor != null) backRightMotor.setPower(0);
    }

    // ----------------------- Calculated Motor Powers -----------------
    double frontLeftMotorSpeed = 0;
    double frontRightMotorSpeed = 0;
    double backLeftMotorSpeed = 0;
    double backRightMotorSpeed = 0;

    // ---------------------- Driver Speed Multipliers ----------------
    double[] drivingSpeeds = {0.25, 0.5, 0.75, 1.0};
    int drivingSpeedsIndex = 1;
    private double speedMultiplier = 0.50; // Default drive speed at 50%

    /**
     * Initializes hardware motors and configures direction and brake behavior with AprilTagAlignment reference.
     */
    public void init(HardwareMap hwMap, AprilTagAlignment aprilTagAlignment) {
        BulkReads.enable(hwMap);
        this.aprilTagAlignment = aprilTagAlignment;
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void init(HardwareMap hwMap) {
        BulkReads.enable(hwMap);
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

//    public void init(HardwareMap hwMap, AprilTagAlignment aprilTagAlignment, OdometryMovement odometryMovement){
//        this.aprilTagAlignment = aprilTagAlignment;
//        this.odometryMovement = odometryMovement;
//        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
//        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
//        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
//        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");
//
//        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }

    /**
     * Initializes hardware motors and configures direction and brake behavior with OdometryMovement reference.
     */
    public void init(HardwareMap hwMap, OdometryMovement odometryMovement) {
        BulkReads.enable(hwMap);
        this.odometryMovement = odometryMovement;
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Drives the robot in Mecanum kinematics using forward (y), strafe (x), and turn (rx) values.
     * Applies multipliers and added offsets from active vision or odometry subsystems before outputting motor power.
     *
     * @param y  Forward/backward stick input (-1.0 to 1.0)
     * @param x  Strafe left/right stick input (-1.0 to 1.0)
     * @param rx Rotation turn stick input (-1.0 to 1.0)
     */
    public void drive(double y, double x, double rx) {
        if ((aprilTagAlignment != null) || (odometryMovement != null)) {
            // Apply AprilTag vision alignment multipliers and rotational offsets
            if (aprilTagAlignment != null) {
                rx *= aprilTagAlignment.getRotationMultiplier();
                // Auto-rotate power is absolute: undo the driver speed setting applied below, so aim tuning does
                // not depend on the drive speed.
                rx += aprilTagAlignment.getAddedRotation() / Math.max(0.05, speedMultiplier);

//                x *= aprilTagAlignment.getStrafeMultiplier();
//                x += aprilTagAlignment.getAddedStrafe();
            }
            // Apply Odometry movement multipliers and positional offsets
            if (odometryMovement != null) {
                x *= odometryMovement.getStrafeMultiplier();
                x += odometryMovement.getAddedStrafe();
                y *= odometryMovement.getForwardMultiplier();
                y += odometryMovement.getAddedForward();
            }
        }

        // Standard Mecanum drive kinematics equations
        frontLeftMotorSpeed = -y + x + rx;
        backLeftMotorSpeed = -y - x + rx;
        frontRightMotorSpeed = -y - x - rx;
        backRightMotorSpeed = -y + x - rx;

        // Apply global speed multiplier and write power to physical motors
        frontLeftMotor.setPower(frontLeftMotorSpeed * speedMultiplier);
        backLeftMotor.setPower(backLeftMotorSpeed * speedMultiplier);
        frontRightMotor.setPower(frontRightMotorSpeed * speedMultiplier);
        backRightMotor.setPower(backRightMotorSpeed * speedMultiplier);
    }

    /**
     * Sets global drive speed scaling factor (e.g. 0.25, 0.50, 0.75, 1.0).
     */
    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = multiplier;
    }

    /**
     * @return Currently active global drive speed scaling factor.
     */
    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    /**
     * Cycles through predefined driver speed steps (25%, 50%, 75%, 100%).
     */
    public void switchSpeed() {
        drivingSpeedsIndex = (drivingSpeedsIndex + 1) % drivingSpeeds.length;
        setSpeedMultiplier(drivingSpeeds[drivingSpeedsIndex]);
    }
}

