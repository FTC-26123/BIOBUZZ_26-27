package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.math.RoundingMode;
import java.text.DecimalFormat;
@Disabled
@TeleOp(name = "TeleOp_Beta_1controller")
public class TeleOp_BETA_1controller extends OpMode {

    public final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.55F;
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotorEx launcher;
    public DcMotor index;
    public DcMotor intake;

    public Servo light;

    public CRServo gate;

    public float frontLeftMotorSpeed = 0;
    public float frontRightMotorSpeed = 0;
    public float backLeftMotorSpeed = 0;
    public float backRightMotorSpeed = 0;

    long dualVelocity = 1200;
    boolean gamepadBWasPressed = false;
    boolean shooterIsBusy = false;

    boolean intaking = false;

    public ElapsedTime lightTimer = new ElapsedTime();

    public void checkForCompEnd(boolean check) {
        if (check) {requestOpModeStop();}
    }

    public void update() {
//     Robot Motor Power Limits
        frontLeftMotor.setPower(frontLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        frontRightMotor.setPower(frontRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backLeftMotor.setPower(backLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backRightMotor.setPower(backRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
    }

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        index = hardwareMap.get(DcMotor.class, "windmill");
        intake = hardwareMap.get(DcMotor.class, "intake");

        light = hardwareMap.get(Servo.class, "light");

        gate = hardwareMap.get(CRServo.class, "gate");

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        index.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(5);

    }

    @Override
    public void loop() {

        frontLeftMotorSpeed = 0;
        frontRightMotorSpeed = 0;
        backLeftMotorSpeed = 0;
        backRightMotorSpeed = 0;

        float left_stick_x = gamepad2.left_stick_x;
        float left_stick_y = gamepad2.left_stick_y;
        float right_stick_x = gamepad2.right_stick_x;

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

        if (gamepad2.a && !gamepad2.start && !gamepad1.start) {
            index.setPower(1);
            gate.setPower(1);
        } else if (gamepad2.b && !gamepad2.start && !gamepad1.start) {
            index.setPower(-1);
            intake.setPower(-1);
            gate.setPower(-1);
            gamepadBWasPressed = true;
        } else if (!gamepad2.a && !gamepad2.b) {
            index.setPower(0);
            gate.setPower(0);
        } if (!gamepad2.b && gamepadBWasPressed) {
            if (!intaking) intake.setPower(0);
            else intake.setPower(1);
            gamepadBWasPressed = false;
        }

        DecimalFormat decimalFormat = new DecimalFormat("#,###");
        decimalFormat.setRoundingMode(RoundingMode.HALF_UP);

        telemetry.addData("Target Launcher Velocity", dualVelocity);
        telemetry.addLine("Launcher Velocity: " + decimalFormat.format(launcher.getVelocity()) + " ... Launcher RPM: " + decimalFormat.format(launcher.getVelocity() / 28 * 60) + "RPM");


        if (gamepad2.left_trigger > 0.4) {
            launcher.setVelocity(dualVelocity);
            shooterIsBusy = true;
        } if (gamepad2.left_bumper) {
            launcher.setVelocity(0);
            shooterIsBusy = false;
        } if (shooterIsBusy && launcher.getVelocity() > (dualVelocity - 45)) {
            light.setPosition(0.47);
            telemetry.addLine("Correct Shooter Power");
        } else if (shooterIsBusy && launcher.getVelocity() < (dualVelocity - 45)) {
            light.setPosition(0.29);
            telemetry.addLine("Low Shooter Power");
        } else if (shooterIsBusy && launcher.getVelocity() > (dualVelocity + 45)) {
            light.setPosition(0.8);
            telemetry.addLine("High Shooter Power");
        } else if (!shooterIsBusy) {
            light.setPosition(0);
        }

        if (gamepad2.right_trigger > 0.4 && !gamepad2.b) {
            intake.setPower(1);
            intaking = true;
        } if (gamepad2.right_bumper) {
            intake.setPower(0);
            intaking = false;
        }

        telemetry.addLine("Gamepad2.dpad_up -> Velocity 1,460");
        if (gamepad2.dpad_up) {
            dualVelocity = 1460;
            if (shooterIsBusy) launcher.setVelocity(dualVelocity);
        }

        telemetry.addLine("Gamepad2.dpad_down -> Velocity 1,200");
        if (gamepad2.dpad_down) {
            dualVelocity = 1200;
            if (shooterIsBusy) launcher.setVelocity(dualVelocity);
        }

        telemetry.addData("Right Launcher Amps", launcher.getCurrent(CurrentUnit.AMPS));

        checkForCompEnd(false);

        telemetry.update();
        update();

    }
}