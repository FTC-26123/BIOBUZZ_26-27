package org.firstinspires.ftc.teamcode.auton;

import static java.lang.Thread.sleep;


import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "RedWall", group = "Autonomous")
public class redWall extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private int pathState;
    private Timer pathTimer;
    private Paths paths;
    public DcMotorEx launcher;
    public DcMotor index;
    public DcMotor intake;
    public Servo light1;
    public Servo gate;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime velTim = new ElapsedTime();

    public void wait(int milliseconds) {
        timer.reset();
        while (true) {
            if (timer.milliseconds() > milliseconds) {break;}
        }
    }

    public void velCheck(int minVelocity) {
        while(launcher.getVelocity() < minVelocity) wait(10);
    }

    @Override
    public void init() {

        light1 = hardwareMap.get(Servo.class, "light");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        index = hardwareMap.get(DcMotor.class, "windmill");
        intake = hardwareMap.get(DcMotor.class, "intake");

        gate = hardwareMap.get(Servo.class, "gate");

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);;
        index.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);


        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(121.254, 125.978, Math.toRadians(37)));

        paths = new Paths(follower);



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

    /** ---------------- PATH DEFINITIONS ---------------- **/

    public static class Paths {
        public PathChain shootpreload;
        public PathChain Intake1st;
        public PathChain go1stSet;
        public PathChain Shoot1st;
        public PathChain intake2nd;
        public PathChain go2ndSet;
        public PathChain Shoot2nd;
        public PathChain Intake3rd;
        public PathChain go3rdSet;
        public PathChain Shoot3rd;

        public Paths(Follower follower) {
            shootpreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(121.254, 125.978),

                                    new Pose(85.136, 89.991)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(41))

                    .build();

            Intake1st = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(85.136, 89.991),
                                    new Pose(88.185, 68.938),
                                    new Pose(88.066, 85.006),
                                    new Pose(104.987, 83.801)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(41), Math.toRadians(0))

                    .build();

            go1stSet = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(104.987, 83.801),

                                    new Pose(126.039, 83.478)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Shoot1st = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.039, 83.478),

                                    new Pose(85.281, 89.905)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(41))

                    .build();

            intake2nd = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(85.281, 89.905),
                                    new Pose(76.183, 63.369),
                                    new Pose(102.729, 59.516)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(41), Math.toRadians(0))

                    .build();

            go2ndSet = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.729, 59.516),

                                    new Pose(133.029, 59.452)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Shoot2nd = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(133.029, 59.452),
                                    new Pose(95.566, 67.945),
                                    new Pose(85.262, 89.939)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(41))

                    .build();

            Intake3rd = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(85.262, 89.939),
                                    new Pose(75.412, 55.290),
                                    new Pose(79.458, 35.894),
                                    new Pose(103.818, 35.215)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(41), Math.toRadians(0))

                    .build();

            go3rdSet = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(103.818, 35.215),

                                    new Pose(133.922, 35.396)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Shoot3rd = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(133.922, 35.396),
                                    new Pose(90.445, 72.630),
                                    new Pose(124.035, 70.225)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))

                    .build();
        }
    }


    /** ---------------- STATE MACHINE ---------------- **/
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {

            case 0:
                if(!follower.isBusy()) {

                    launcher.setVelocity(1023);
                    intake.setPower(0.7);
                    follower.followPath(paths.shootpreload, 0.8, true);
                    setPathState(1);
                }
                break;

            case 1:
                if(!follower.isBusy()) {

                    intake.setPower(0.8);
                    gate.setPosition(0.27);

                    velCheck(1023-50);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);

                    velCheck(1023-50);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);

                    velCheck(1023-50);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);

                    gate.setPosition(0.50);
                    setPathState(2);

                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    intake.setPower(1);
                    index.setPower(0.5);
                    follower.followPath(paths.Intake1st, 0.85, false);
                    launcher.setVelocity(1160);
                    setPathState(100);
                }
                break;

            case 100:
                if (!follower.isBusy()) {
                    follower.followPath(paths.go1stSet, 0.325, true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    intake.setPower(0.5);
                    index.setPower(0);
                    follower.followPath(paths.Shoot1st);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {

                    gate.setPosition(0.27);
                    velCheck(1110);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);

                    velCheck(1110);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);

                    velCheck(1110);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);
                    gate.setPosition(0.50);

                    setPathState(5);
                }
                break;


            case 5:
                if(!follower.isBusy()) {
                    intake.setPower(1);
                    index.setPower(0.5);
                    follower.followPath(paths.intake2nd,0.85,false);
                    setPathState(101);
                }
                break;

            case 101:
                if (!follower.isBusy()) {
                    follower.followPath(paths.go2ndSet, 0.325, true);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    intake.setPower(0.5);
                    index.setPower(0);
                    follower.followPath(paths.Shoot2nd, 1.0, true);
                    setPathState(7);
                }
                break;

            case 7:
                if(!follower.isBusy()) {

                    gate.setPosition(0.27);
                    velCheck(1110);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);

                    velCheck(1110);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);

                    velCheck(1110);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);
                    gate.setPosition(0.50);

                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    intake.setPower(1);
                    index.setPower(0.5);
                    follower.followPath(paths.Intake3rd, 1, false);
                    setPathState(102);
                }

            case 102:
                if (!follower.isBusy()) {
                    follower.followPath(paths.go3rdSet, 0.325, true);
                    setPathState(9);
                }
                break;

            case 9:
                if(!follower.isBusy()) {

                    follower.followPath(paths.Shoot3rd, 1.0, true);
                    launcher.setVelocity(0);
                    index.setPower(0);
                    intake.setPower(0);

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
