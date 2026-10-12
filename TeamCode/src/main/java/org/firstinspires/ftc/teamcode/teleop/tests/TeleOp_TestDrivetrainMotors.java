package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Diagnostic OpMode to test each Mecanum drivetrain motor individually at 0.7 power.
 * Press Gamepad 1 A for Front Left, B for Front Right, X for Back Left, Y for Back Right.
 * Pull Gamepad 1 Right Trigger to turn ALL motors OFF immediately.
 */
@TeleOp
public class TeleOp_TestDrivetrainMotors extends OpMode {

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private static final double TEST_POWER = 0.7;

    private String activeMotor = "None";

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

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Emergency / Safety Stop on Right Trigger
        if (gamepad1.right_trigger > 0.3) {
            stopAllMotors();
            activeMotor = "ALL MOTORS STOPPED (Right Trigger)";
        }
        // Test individual motors at 0.7 power
        else if (gamepad1.a) {
            stopAllMotors();
            frontLeftMotor.setPower(TEST_POWER);
            activeMotor = "Front Left Motor (frontLeftMotor)";
        } else if (gamepad1.b) {
            stopAllMotors();
            frontRightMotor.setPower(TEST_POWER);
            activeMotor = "Front Right Motor (frontRightMotor)";
        } else if (gamepad1.x) {
            stopAllMotors();
            backLeftMotor.setPower(TEST_POWER);
            activeMotor = "Back Left Motor (backLeftMotor)";
        } else if (gamepad1.y) {
            stopAllMotors();
            backRightMotor.setPower(TEST_POWER);
            activeMotor = "Back Right Motor (backRightMotor)";
        } else if (gamepad1.right_trigger_pressed) {
            stopAllMotors();
            activeMotor = "None";
        }

        // Telemetry
        telemetry.addLine("================ CONTROLS ================");
        telemetry.addLine("G1 A : Front Left Motor (frontLeftMotor)");
        telemetry.addLine("G1 B : Front Right Motor (frontRightMotor)");
        telemetry.addLine("G1 X : Back Left Motor (backLeftMotor)");
        telemetry.addLine("G1 Y : Back Right Motor (backRightMotor)");
        telemetry.addLine("G1 Right Trigger : STOP ALL MOTORS");

        telemetry.addLine("================ STATUS ================");
        telemetry.addData("Active Motor", activeMotor);
        telemetry.addData("Test Power", TEST_POWER);
        telemetry.addData("FL Power", "%.2f", frontLeftMotor.getPower());
        telemetry.addData("FR Power", "%.2f", frontRightMotor.getPower());
        telemetry.addData("BL Power", "%.2f", backLeftMotor.getPower());
        telemetry.addData("BR Power", "%.2f", backRightMotor.getPower());

        telemetry.setMsTransmissionInterval(30);
        telemetry.update();
    }

    private void stopAllMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
