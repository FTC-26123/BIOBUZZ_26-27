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

@Autonomous(name = "redFar", group = "Autonomous")
public class redFar extends OpMode {

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
        follower.setStartingPose(new Pose(80.822, 8.340, Math.toRadians(90)));

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
        public PathChain shoot1st;
        public PathChain intake2nd;
        public PathChain go2ndSet;
        public PathChain shoot2nd;
        public PathChain intake3rd;
        public PathChain go3rdSet;
        public PathChain shoot3rd;
        public PathChain wallIntake1;

        public Paths(Follower follower) {
            shoot1st = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(80.822, 8.340),

                                    new Pose(82.522, 16.449)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(64))

                    .build();

            intake2nd = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(82.522, 16.449),
                                    new Pose(96.321, 32.661),
                                    new Pose(81.081, 35.126),
                                    new Pose(98.050, 35.532)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(64), Math.toRadians(0))

                    .build();

            go2ndSet = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(98.050, 35.532),

                                    new Pose(129.984, 35.426)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shoot2nd = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.984, 35.426),

                                    new Pose(82.904, 16.490)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(64))

                    .build();

            intake3rd = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(82.904, 16.490),
                                    new Pose(75.498, 61.900),
                                    new Pose(94.233, 59.407)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(64), Math.toRadians(0))

                    .build();

            go3rdSet = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(94.233, 59.407),

                                    new Pose(133.742, 59.201)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shoot3rd = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.742, 59.201),

                                    new Pose(82.635, 16.662)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(64))

                    .build();

            wallIntake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.635, 16.662),

                                    new Pose(102.087, 43.797)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(64), Math.toRadians(64))

                    .build();
        }
    }


    /** ---------------- STATE MACHINE ---------------- **/
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {

            case 0:
                if(!follower.isBusy()) {

                    launcher.setVelocity(1430);
                    intake.setPower(0.7);
                    follower.followPath(paths.shoot1st);
                    setPathState(1);
                }
                break;

            case 1:
                if(!follower.isBusy()) {

                    intake.setPower(0.8);
                    gate.setPosition(0.27);

                    wait(1000);

                    while (launcher.getVelocity() < 1400) wait(10);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);

                    velCheck(1400);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);

                    velCheck(1400);
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
                    follower.followPath(paths.intake2nd, 1.00, false);
                    launcher.setVelocity(1430);
                    setPathState(100);
                }
                break;

            case 100:
                if (!follower.isBusy()) {
                    follower.followPath(paths.go2ndSet, 0.325, false);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    intake.setPower(0.5);
                    index.setPower(0);
                    follower.followPath(paths.shoot2nd);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {

                    gate.setPosition(0.27);
                    velCheck(1400);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);


                    velCheck(1400);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);

                    velCheck(1400);
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
                    follower.followPath(paths.intake3rd,1.00,false);
                    setPathState(101);
                }
                break;

            case 101:
                if (!follower.isBusy()) {
                    follower.followPath(paths.go3rdSet, 0.325, false);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    intake.setPower(0.5);
                    index.setPower(0);
                    follower.followPath(paths.shoot3rd);
                    setPathState(7);
                }
                break;

            case 7:
                if(!follower.isBusy()) {

                    gate.setPosition(0.27);
                    velCheck(1400);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);

                    velCheck(1400);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);

                    velCheck(1400);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);
                    gate.setPosition(0.50);

                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    launcher.setVelocity(0);
                    index.setPower(0);
                    intake.setPower(0);
                    follower.followPath(paths.wallIntake1, 1.00, true);
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
