package org.firstinspires.ftc.teamcode.teleop;


import static java.lang.Thread.sleep;

//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
public class TeleOp_BaseCode extends OpMode {
    //Initializing and declaring all variables/motors
    private final ElapsedTime runtime = new ElapsedTime();
    public Shooter shooter = new Shooter();
    public Intake intake = new Intake();
    public MecanumDrive driveTrain = new MecanumDrive();
    public ElapsedTime TeleOpRuntime = new ElapsedTime();
    public Limelight limelight = new Limelight();
    public AprilTagAlignment aprilTagAlignment = new AprilTagAlignment();

    @Override
    public void init() {
        limelight.init(hardwareMap);
        aprilTagAlignment.init(limelight, runtime);
        driveTrain.init(hardwareMap, aprilTagAlignment);
        intake.init(hardwareMap);
        shooter.init(hardwareMap, intake);

    }

    @Override
    public void start() {
        runtime.reset();
        TeleOpRuntime.reset();
        limelight.start();
    }

    @Override
    public void loop() {

        // -------------- Limelight
        limelight.update();

        // -------------- AprilTagAlignment
        aprilTagAlignment.update();
        handleAutoRotation(gamepad1);
        handleApriltagAlignmentIncrements(gamepad1);
        //handleAutoStrafe(gamepad1);


        // ---------------- Control Drive
        handleDpadSpeedSwitching(gamepad1);
        driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


        // ---------------- Control Intake

        handleManualIntakeControl(gamepad2);

        // ---------------- Control Shooter

        handleManualShootingControl(gamepad2);
        handlePresets(gamepad2);
        handleManualGateControl(gamepad2);
        shooter.update(gamepad2);





        telemetry.addData("Robot Speed Multiplier", driveTrain.getSpeedMultiplier());

        telemetry.addData("Target Launch Power", shooter.getTargetLaunchPower());
        telemetry.addData("Current Motor Speed", shooter.getCurrentSpeed());
        telemetry.addData("Shooter Status", shooter.getShooterStatus());
        telemetry.addData("Gate Status", shooter.getGateStatus());
        telemetry.addData("Intake Status", intake.getIntakeStatus());


        telemetry.addData("Runtime:", TeleOpRuntime.seconds());
        telemetry.setMsTransmissionInterval(30);
        telemetry.update();
    }

    private void handleManualShootingControl(Gamepad gamepad2){
        if (gamepad2.right_trigger>0.5) {
            shooter.start();
        } else if (gamepad2.right_bumper){
            shooter.stop();
        }
    }
    private void handlePresets(Gamepad gamepad2){
        if(gamepad2.dpad_up){
            //Very far
            shooter.switchSpeed(2000);
        }
        if(gamepad2.dpad_right){
            //Far
            shooter.switchSpeed(1500);
        }
        if(gamepad2.dpad_down){
            //Close
            shooter.switchSpeed(1000);
        }
    }

    private void handleManualGateControl(Gamepad gamepad2){
        if (gamepad2.x) {
            shooter.openGate();
        }
        else if (gamepad2.y) {
            shooter.closeGate();
        }
    }

    private void handleManualIntakeControl(Gamepad gamepad2) {
        if (gamepad2.left_trigger > 0.5) {
            //Starting intake
            intake.start();
        } else if (gamepad2.back) {
            //Reversing intake
            intake.out();
        } else if (gamepad2.left_bumper) {
            //Stopping intake
            intake.stop();
        }
    }

    private void handleDpadSpeedSwitching(Gamepad gamepad1){
        if (gamepad1.dpadRightWasPressed()){
            driveTrain.switchSpeed();
        }
    }

    private void handleApriltagAlignmentIncrements(Gamepad gamepad1){
        if (gamepad1.dpadUpWasPressed()){
            aprilTagAlignment.addIndexValue();
        }
        if (gamepad1.dpadDownWasPressed()){
            aprilTagAlignment.subIndexValue();
        }
        if (gamepad1.dpadLeftWasPressed()){
            aprilTagAlignment.switchPIDModifyingValue();
        }
        if (gamepad1.bWasPressed()){
            aprilTagAlignment.switchStepSize();
        }
    }

    private void handleAutoRotation(Gamepad gamepad1){
        if (gamepad1.left_trigger>0.3) {
            aprilTagAlignment.autoRotate();
        } else {
            aprilTagAlignment.setManualRotation();
        }
    }

    private void handleAutoStrafe(Gamepad gamepad1){
        if (gamepad1.left_bumper) {
            aprilTagAlignment.autoStrafe();
        } else {
            aprilTagAlignment.setManualStrafing();
        }
    }
}


