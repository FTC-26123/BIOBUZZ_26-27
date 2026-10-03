package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.OdometryMovement;

@TeleOp
public class TeleOp_OdometryTesting extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    public MecanumDrive driveTrain = new MecanumDrive();
    public OdometryMovement odometryMovement = new OdometryMovement();

    @Override
    public void init() {
        driveTrain.init(hardwareMap, odometryMovement);
        odometryMovement.init(hardwareMap, driveTrain);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        driveTrain.update(gamepad1);
        odometryMovement.update(gamepad1);

        driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // --- Odometry Pose Telemetry ---
        telemetry.addLine("---------------- Odometry Pose ----------------");
        telemetry.addData("X Position (in)", "%.2f", odometryMovement.getHorizantalValue());
        telemetry.addData("Y Position (in)", "%.2f", odometryMovement.getVerticalValue());
        telemetry.addData("Heading (°)", "%.2f", odometryMovement.getHeadingDegrees());

        // --- Distance Control Controls & Status ---
        telemetry.addLine("------------ Distance Driving Controls ------------");
        telemetry.addData("Distance to move (in) [G1 DpadUp/Down]", "%.1f", odometryMovement.getInputDistance());
        telemetry.addData("Direction [G1 A]", odometryMovement.getSelectedDirection());
        telemetry.addData("Step Size (in) [G1 B]", odometryMovement.getSelectedIncrement());
        telemetry.addData("Odometry Status [G1 RightTrigger]", odometryMovement.getStatus());
        telemetry.addData("Error", odometryMovement.getError());
        telemetry.addData("Target Distance", odometryMovement.getTargetDistance());

        // --- Drive Train Status ---
        telemetry.addLine("----------------- Drive Status -----------------");
        telemetry.addData("Drive Speed Multiplier [G1 DpadRight]", driveTrain.getSpeedMultiplier());
        telemetry.addData("Runtime (s)", "%.2f", runtime.seconds());



        telemetry.setMsTransmissionInterval(30);
        telemetry.update();
    }
}
