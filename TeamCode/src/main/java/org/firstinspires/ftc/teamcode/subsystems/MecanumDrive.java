package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {


    // ---------------------- Getting Hardware ----------------
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private AprilTagAlignment aprilTagAlignment;

    private  OdometryMovement odometryMovement;


    // ----------------------- Motor Variables -----------------
    double frontLeftMotorSpeed = 0;
    double frontRightMotorSpeed = 0;
    double backLeftMotorSpeed = 0;
    double backRightMotorSpeed = 0;


    // ---------------------- Dpad Increments(Speed) ------------
    double[] drivingSpeeds = {0.25,0.5,0.75,1.0};
    int drivingSpeedsIndex = 1;

    private double speedMultiplier = 0.50;


    public void init(HardwareMap hwMap, AprilTagAlignment aprilTagAlignment){
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
    public void init(HardwareMap hwMap, OdometryMovement odometryMovement){
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

    public void drive(double y, double x, double rx) {
        if ((aprilTagAlignment != null) || (odometryMovement != null)) {
            if (aprilTagAlignment != null) {
                rx *= aprilTagAlignment.getRotationMultiplier();
                rx += aprilTagAlignment.getAddedRotation();

//                x *= aprilTagAlignment.getStrafeMultiplier();
//                x += aprilTagAlignment.getAddedStrafe();
            }
            if (odometryMovement != null) {
                x *= odometryMovement.getStrafeMultiplier();
                x += odometryMovement.getAddedStrafe();
                y *= odometryMovement.getForwardMultiplier();
                y += odometryMovement.getAddedForward();
            }


        }

        frontLeftMotorSpeed = -y + x + rx;
        backLeftMotorSpeed = -y - x + rx;
        frontRightMotorSpeed = -y - x - rx;
        backRightMotorSpeed = -y + x - rx;

        frontLeftMotor.setPower(frontLeftMotorSpeed*speedMultiplier);
        backLeftMotor.setPower(backLeftMotorSpeed*speedMultiplier);
        frontRightMotor.setPower(frontRightMotorSpeed*speedMultiplier);
        backRightMotor.setPower(backRightMotorSpeed*speedMultiplier);
    }
    public void setSpeedMultiplier(double multiplier){
        speedMultiplier = multiplier;
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }
    public void update(Gamepad gamepad1){
        handleDpadSpeedSwitching(gamepad1);
    }
    private void handleDpadSpeedSwitching(Gamepad gamepad1){
        if (gamepad1.dpadRightWasPressed()){
            drivingSpeedsIndex = (drivingSpeedsIndex+1) % drivingSpeeds.length;
            setSpeedMultiplier(drivingSpeeds[drivingSpeedsIndex]);
        }

    }
}
