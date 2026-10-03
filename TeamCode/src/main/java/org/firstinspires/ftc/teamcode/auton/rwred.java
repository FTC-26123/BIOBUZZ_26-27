package org.firstinspires.ftc.teamcode.auton;


import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Autonomous (name = "redFarBasic", group = "Autonomous")

public class rwred extends OpMode {
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private int pathState;

    private Timer pathTimer;

    public DcMotorEx launcher;
    public DcMotor index;
    public DcMotor intake;

    public Servo light1;

    public Servo gate;
    private rwred.Paths paths;

    ElapsedTime timer = new ElapsedTime();


    public void wait(int milliseconds) {

        timer.reset();
        while (true) {
            if (timer.milliseconds() > milliseconds) {
                break;
            }


        }
    }



    @Override
    public void init() {
        light1 = hardwareMap.get(Servo.class, "light1");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        index = hardwareMap.get(DcMotor.class, "windmill");
        intake = hardwareMap.get(DcMotor.class, "intake");

        gate = hardwareMap.get(Servo.class,"gate");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        index.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(82.203, 8.944, Math.toRadians(90)));
        paths = new rwred.Paths(follower);
    }

    @Override
    public void start() {
        pathState = 0;
        pathTimer.resetTimer();
        timer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.203, 8.944),

                                    new Pose(87.869, 19.239)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(64))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.869, 19.239),

                                    new Pose(108.341, 10.364)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(64), Math.toRadians(0))

                    .build();
        }
    }


    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    gate.setPosition(0.27);
                    launcher.setVelocity(1430);
                    follower.followPath(paths.Path1, 0.25, true);
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path2, 0.25, true);
                    setPathState(2);
                    while (launcher.getVelocity() < 1400) wait(10);

                    index.setPower(0.5);
                    wait(10000);
                    index.setPower(0);

                }
                break;
            case 2:
                if (!follower.isBusy()){
                    launcher.setVelocity(0);
                    index.setPower(0);
                    gate.setPosition(0.50);
                    setPathState(-1);
                }
                break;
        }
    }

    private void setPathState(int s) {
        pathState = s;
        pathTimer.resetTimer();
    }
}