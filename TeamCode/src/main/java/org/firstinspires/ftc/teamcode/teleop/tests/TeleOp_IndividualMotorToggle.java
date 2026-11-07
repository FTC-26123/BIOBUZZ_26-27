package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Diagnostic TeleOp to toggle individual motors ON and OFF using dedicated single-button presses.
 */
@TeleOp
public class TeleOp_IndividualMotorToggle extends OpMode {

    // ---------------------- Hardware Motor Objects ----------------------
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor intake;
    private DcMotor windmill;
    private DcMotorEx shooter;

    // ---------------------- Toggle States ----------------------
    private boolean frontLeftOn = false;
    private boolean frontRightOn = false;
    private boolean backLeftOn = false;
    private boolean backRightOn = false;
    private boolean intakeOn = false;
    private boolean windmillOn = false;
    private boolean shooterOn = false;

    // Default power and velocity test constants
    private static final double DRIVE_POWER = 0.5;
    private static final double INTAKE_POWER = 0.7;
    private static final double WINDMILL_POWER = 1.0;
    private static final double SHOOTER_VELOCITY = 500.0; // ticks/sec

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        windmill = hardwareMap.get(DcMotor.class, "windmill");
        windmill.setDirection(DcMotorSimple.Direction.FORWARD);
        windmill.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Toggle individual motor states on single button press
        if (gamepad1.xWasPressed()) {
            frontLeftOn = !frontLeftOn;
        }
        if (gamepad1.yWasPressed()) {
            frontRightOn = !frontRightOn;
        }
        if (gamepad1.aWasPressed()) {
            backLeftOn = !backLeftOn;
        }
        if (gamepad1.bWasPressed()) {
            backRightOn = !backRightOn;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            intakeOn = !intakeOn;
        }
        if (gamepad1.dpadRightWasPressed()) {
            windmillOn = !windmillOn;
        }
        if (gamepad1.dpadUpWasPressed()) {
            shooterOn = !shooterOn;
        }

        // Global Stop on Dpad Down or Right Trigger > 0.3
        if (gamepad1.dpadDownWasPressed() || gamepad1.right_trigger > 0.3) {
            frontLeftOn = false;
            frontRightOn = false;
            backLeftOn = false;
            backRightOn = false;
            intakeOn = false;
            windmillOn = false;
            shooterOn = false;
        }

        // Output motor powers based on active toggle states
        frontLeftMotor.setPower(frontLeftOn ? DRIVE_POWER : 0.0);
        frontRightMotor.setPower(frontRightOn ? DRIVE_POWER : 0.0);
        backLeftMotor.setPower(backLeftOn ? DRIVE_POWER : 0.0);
        backRightMotor.setPower(backRightOn ? DRIVE_POWER : 0.0);

        intake.setPower(intakeOn ? INTAKE_POWER : 0.0);
        windmill.setPower(windmillOn ? WINDMILL_POWER : 0.0);
        shooter.setVelocity(shooterOn ? SHOOTER_VELOCITY : 0.0);

        // Telemetry Output
        telemetry.addLine("================ MOTOR TOGGLE STATUS ================");
        telemetry.addData("front left(G1 X)", frontLeftOn ? "ON" : "OFF");
        telemetry.addData("front right(G1 Y)", frontRightOn ? "ON" : "OFF");
        telemetry.addData("back left(G1 A)", backLeftOn ? "ON" : "OFF");
        telemetry.addData("back right(G1 B)", backRightOn ? "ON" : "OFF");
        telemetry.addData("intake(G1 Dpad Left)", intakeOn ? "ON" : "OFF");
        telemetry.addData("windmill(G1 Dpad Right)", windmillOn ? "ON" : "OFF");
        telemetry.addData("shooter(G1 Dpad Up)", shooterOn ? "ON" : "OFF");
        telemetry.addLine("---------------------------------------------------");
        telemetry.addLine("G1 Dpad Down or RT : TURN ALL MOTORS OFF");

        telemetry.setMsTransmissionInterval(30);
        telemetry.update();
    }
}
