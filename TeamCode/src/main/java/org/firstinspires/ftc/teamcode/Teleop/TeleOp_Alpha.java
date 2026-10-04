package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name = "TeleOp_ALPHA")
public class TeleOp_Alpha extends OpMode {

    public final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.55F;

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    public float frontLeftMotorSpeed = 0;
    public float frontRightMotorSpeed = 0;
    public float backLeftMotorSpeed = 0;
    public float backRightMotorSpeed = 0;

    public Limelight3A limelight;

    // PID values
    double kP = 0.02;
    double kI = 0.0;
    double kD = 0.0;

    double integral = 0;
    double previousError = 0;

    final double TARGET_TX = 0.0;
    final double TX_TOLERANCE = 1.0;
    final double MAX_PID_TURN = 0.4;

    ElapsedTime pidTimer = new ElapsedTime();

    public void update() {
        frontLeftMotor.setPower(
                frontLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP
        );

        frontRightMotor.setPower(
                frontRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP
        );

        backLeftMotor.setPower(
                backLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP
        );

        backRightMotor.setPower(
                backRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP
        );
    }

    @Override
    public void init() {

        // Motors
        frontLeftMotor =
                hardwareMap.get(DcMotor.class, "frontLeftMotor");

        frontRightMotor =
                hardwareMap.get(DcMotor.class, "frontRightMotor");

        backLeftMotor =
                hardwareMap.get(DcMotor.class, "backLeftMotor");

        backRightMotor =
                hardwareMap.get(DcMotor.class, "backRightMotor");

        frontRightMotor.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        // Limelight
        limelight =
                hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        pidTimer.reset();

        telemetry.addLine("Robot Ready");
        telemetry.addLine("Hold A to auto-align to AprilTag");
        telemetry.update();
    }

    @Override
    public void loop() {

        frontLeftMotorSpeed = 0;
        frontRightMotorSpeed = 0;
        backLeftMotorSpeed = 0;
        backRightMotorSpeed = 0;

        float left_stick_x = gamepad1.left_stick_x;
        float left_stick_y = gamepad1.left_stick_y;
        float right_stick_x = gamepad1.right_stick_x;

        // Forward / backward
        if (left_stick_y != 0) {
            frontLeftMotorSpeed = -left_stick_y;
            frontRightMotorSpeed = -left_stick_y;
            backLeftMotorSpeed = -left_stick_y;
            backRightMotorSpeed = -left_stick_y;
        }

        // Strafing
        if (left_stick_x != 0) {
            frontLeftMotorSpeed += left_stick_x;
            frontRightMotorSpeed -= left_stick_x;
            backLeftMotorSpeed -= left_stick_x;
            backRightMotorSpeed += left_stick_x;
        }

        // Read Limelight
        LLResult result = limelight.getLatestResult();

        boolean aprilTagFound = false;
        double tx = 0;
        int tagID = -1;

        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> tags =
                    result.getFiducialResults();

            if (tags != null && !tags.isEmpty()) {

                LLResultTypes.FiducialResult tag = tags.get(0);

                aprilTagFound = true;

                tagID = tag.getFiducialId();
                tx = tag.getTargetXDegrees();

                telemetry.addLine("AprilTag Found");
                telemetry.addData("Tag ID", tagID);
                telemetry.addData("TX", "%.2f degrees", tx);
            }
        }

        // Auto-align while holding A
        if (gamepad1.a && aprilTagFound) {

            double error = TARGET_TX - tx;

            double dt = pidTimer.seconds();
            pidTimer.reset();

            if (dt < 0.001) {
                dt = 0.001;
            }

            integral += error * dt;

            integral =
                    Math.max(
                            -50,
                            Math.min(50, integral)
                    );

            double derivative =
                    (error - previousError) / dt;

            previousError = error;

            double turnPower =
                    kP * error
                            + kI * integral
                            + kD * derivative;

            turnPower =
                    Math.max(
                            -MAX_PID_TURN,
                            Math.min(MAX_PID_TURN, turnPower)
                    );

            if (Math.abs(error) < TX_TOLERANCE) {
                turnPower = 0;
                integral = 0;
            }

            frontLeftMotorSpeed += turnPower;
            backLeftMotorSpeed += turnPower;

            frontRightMotorSpeed -= turnPower;
            backRightMotorSpeed -= turnPower;

            telemetry.addLine("Auto Align Active");
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Turn Power", "%.3f", turnPower);
        }

        else {

            // Manual rotation
            if (right_stick_x != 0) {
                frontLeftMotorSpeed += right_stick_x;
                backLeftMotorSpeed += right_stick_x;

                frontRightMotorSpeed -= right_stick_x;
                backRightMotorSpeed -= right_stick_x;
            }

            integral = 0;
            previousError = 0;
            pidTimer.reset();

            if (gamepad1.a && !aprilTagFound) {
                telemetry.addLine("A pressed, but no AprilTag found");
            }
        }

        // Normalize motor powers
        double largest =
                Math.max(
                        Math.max(
                                Math.abs(frontLeftMotorSpeed),
                                Math.abs(frontRightMotorSpeed)
                        ),
                        Math.max(
                                Math.abs(backLeftMotorSpeed),
                                Math.abs(backRightMotorSpeed)
                        )
                );

        if (largest > 1.0) {
            frontLeftMotorSpeed /= largest;
            frontRightMotorSpeed /= largest;
            backLeftMotorSpeed /= largest;
            backRightMotorSpeed /= largest;
        }

        update();

        if (!aprilTagFound) {
            telemetry.addLine("No AprilTag detected");
        }

        telemetry.addData(
                "Auto Align",
                gamepad1.a ? "ON" : "OFF"
        );

        telemetry.update();
    }

    @Override
    public void stop() {

        if (limelight != null) {
            limelight.stop();
        }

        if (frontLeftMotor != null) {
            frontLeftMotor.setPower(0);
        }

        if (frontRightMotor != null) {
            frontRightMotor.setPower(0);
        }

        if (backLeftMotor != null) {
            backLeftMotor.setPower(0);
        }

        if (backRightMotor != null) {
            backRightMotor.setPower(0);
        }
    }
}