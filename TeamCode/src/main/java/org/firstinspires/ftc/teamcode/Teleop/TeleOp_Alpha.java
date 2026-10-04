package org.firstinspires.ftc.teamcode.Teleop;


import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
@TeleOp(name = "TeleOp_ALPHA")
public class TeleOp_Alpha extends OpMode {
    //Initializing and declaring all variables/motors
    public final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.55F;
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    public float frontLeftMotorSpeed = 0;
    public float frontRightMotorSpeed = 0;
    public float backLeftMotorSpeed = 0;
    public float backRightMotorSpeed = 0;




    public void update() {
//     Robot Motor Power Limits
        frontLeftMotor.setPower(frontLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        frontRightMotor.setPower(frontRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backLeftMotor.setPower(backLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backRightMotor.setPower(backRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
    }

    @Override
    public void init() {
        // Motors & Servos
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Limelight


    }


    @Override
    public void loop() {

//        //limelight
//        double tx = 0;
//        double ty = 0;
//        Pose3D botpose = null;
//        double distance = 0;
//        // modify :
//        double shooterVelocity = -1;
//        double k = fudgeFactor;
//        double flywheel = 1.89;
//
//        if (result != null && result.isValid()) {
//            tx = result.getTx();
//            ty = result.getTy();
//            botpose = result.getBotpose();
//
//            // --- Distance Calculation ---
//            double angleToGoalDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + ty;
//            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
//            distance = (APRILTAG_HEIGHT - LIMELIGHT_LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);
//
//            telemetry.addData("distance", distance);
//
//            double g = 386.0; // gravity in in/s^2
//            double theta = Math.toRadians(launchAngleDegrees);
//            double y = goalHeight - launchHeight;
//
//            double denominator = 2 * Math.pow(Math.cos(theta), 2) * (distance * Math.tan(theta) - y);
//            if (denominator <= 0)
//                telemetry.addLine("Too close");
//
//            double velocity = Math.sqrt((g * distance * distance) / denominator) * (1 + k);
//
//            finalvelocity = velocity / flywheel;
//
//            shooter.setVelocity(finalvelocity);
//
//        }
//
//        if (result == null || !result.isValid()) {
//            // Show red even when april tag is not visible to bot
//            telemetry.addLine("No Target Found");
//        }



        // Movement w/ Joysticks
        frontLeftMotorSpeed = 0;
        frontRightMotorSpeed = 0;
        backLeftMotorSpeed = 0;
        backRightMotorSpeed = 0;

        float left_stick_x = gamepad1.left_stick_x;
        float left_stick_y = gamepad1.left_stick_y;
        float right_stick_x = gamepad1.right_stick_x;

//        Forward/Backward Movement
        if (left_stick_y != 0) {
            frontLeftMotorSpeed = -left_stick_y;
            frontRightMotorSpeed = -left_stick_y;
            backLeftMotorSpeed = -left_stick_y;
            backRightMotorSpeed = -left_stick_y;
        }

//        Lateral Movement
        if (left_stick_x != 0) {
            frontLeftMotorSpeed += left_stick_x;
            frontRightMotorSpeed -= left_stick_x;
            backLeftMotorSpeed -= left_stick_x;
            backRightMotorSpeed += left_stick_x;
        }

//        Rotation
        if (right_stick_x != 0) {
            frontLeftMotorSpeed += right_stick_x;
            backLeftMotorSpeed += right_stick_x;
            frontRightMotorSpeed -= right_stick_x;
            backRightMotorSpeed -= right_stick_x;
        }
    }
}