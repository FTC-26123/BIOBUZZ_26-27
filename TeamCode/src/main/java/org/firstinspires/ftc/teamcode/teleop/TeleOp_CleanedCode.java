package org.firstinspires.ftc.teamcode.teleop;


import static java.lang.Thread.sleep;

//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
public class TeleOp_CleanedCode extends OpMode {
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
        aprilTagAlignment.init(limelight,runtime);
        driveTrain.init(hardwareMap,aprilTagAlignment);
        intake.init(hardwareMap);
        shooter.init(hardwareMap,intake);

    }

    @Override
    public void start() {
        runtime.reset();
        TeleOpRuntime.reset();
        limelight.start();
    }

    @Override
    public void loop() {
        limelight.update();
        aprilTagAlignment.update(gamepad1);
        driveTrain.drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);

        intake.update(gamepad2);
        shooter.update(gamepad2);
        driveTrain.update(gamepad1);


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
}


