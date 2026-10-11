package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
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
        this.aprilTagAlignment = aprilTagAlignment;
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

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
        this.odometryMovement = odometryMovement;
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

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
                rx += aprilTagAlignment.getAddedRotation();

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
