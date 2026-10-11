package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class red_forward extends LinearOpMode {

    private Follower follower;
    private Limelight3A limelight;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58, 8, 90);
    private final Pose shoot1 = poseFactory.of(58, 20, 90);
    private final Pose shoot2 = poseFactory.of(58, 20, 90);
    private final Pose park = poseFactory.of(12.8411, 92.1374, 256);
    private final Pose parkControl1 = poseFactory.of(27.3674, 48.8626, 0);

    private static final int RED_PIPELINE = 0;

    private static final double TURN_KP = 0.025;

    private static final double MAX_TURN = 0.35;

    private static final double SEARCH_TURN = 0.15;

    private static final double FAST_FORWARD = 0.55;
    private static final double SLOW_FORWARD = 0.25;

    private static final double TURN_ONLY_ERROR = 10.0;
    private static final double SLOW_ERROR = 4.0;

    /*
     * Tune this using telemetry.
     *
     * Bigger bottomY = ball is lower in camera = normally closer.
     *
     * 380 is only a starting example.
     */
    private static final double BALL_CLOSE_BOTTOM_Y = 380;

    public Command autoRoutine() {

        return sequential(

                follow(follower, shoot1()),

                seekRedBalls(),

                goToShoot2FromCurrentPosition(),

                follow(follower, park())
        );
    }

    private Command seekRedBalls() {

        return Command.build()

                .setExecute(() -> {

                    LLResultTypes.ColorResult target = getBestRedTarget();

                    if (target == null) {

                        follower.manual(
                                0,
                                0,
                                SEARCH_TURN
                        );

                        return;
                    }

                    double tx = target.getTargetXDegrees();

                    double turn = TURN_KP * tx;

                    turn = clamp(
                            turn,
                            -MAX_TURN,
                            MAX_TURN
                    );

                    double forward;

                    if (Math.abs(tx) > TURN_ONLY_ERROR) {

                        forward = 0;

                    } else if (Math.abs(tx) > SLOW_ERROR) {

                        forward = SLOW_FORWARD;

                    } else {

                        forward = FAST_FORWARD;
                    }

                    double bottomY = getBottomY(target);

                    /*
                     * Once the balls are close, stop driving forward.
                     * setDone() will finish the command immediately after.
                     */
                    if (bottomY >= BALL_CLOSE_BOTTOM_Y) {

                        forward = 0;
                        turn = 0;
                    }

                    follower.manual(
                            forward,
                            0,
                            turn
                    );
                })

                .setDone(() -> {

                    LLResultTypes.ColorResult target = getBestRedTarget();

                    if (target == null) {
                        return false;
                    }

                    return getBottomY(target) >= BALL_CLOSE_BOTTOM_Y;
                })

                .setEnd(endCondition -> {

                    follower.manual(
                            0,
                            0,
                            0
                    );
                });
    }

    private LLResultTypes.ColorResult getBestRedTarget() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.ColorResult> targets =
                result.getColorResults();

        if (targets == null || targets.isEmpty()) {
            return null;
        }

        LLResultTypes.ColorResult bestTarget = null;
        double biggestArea = -1;

        for (LLResultTypes.ColorResult target : targets) {

            if (target.getTargetArea() > biggestArea) {

                biggestArea = target.getTargetArea();
                bestTarget = target;
            }
        }

        return bestTarget;
    }

    private double getBottomY(
            LLResultTypes.ColorResult target) {

        List<List<Double>> corners =
                target.getTargetCorners();

        if (corners == null || corners.isEmpty()) {
            return target.getTargetYPixels();
        }

        double bottomY =
                Double.NEGATIVE_INFINITY;

        for (List<Double> corner : corners) {

            if (corner.size() < 2) {
                continue;
            }

            double y = corner.get(1);

            if (y > bottomY) {
                bottomY = y;
            }
        }

        if (bottomY == Double.NEGATIVE_INFINITY) {
            return target.getTargetYPixels();
        }

        return bottomY;
    }

    private Command goToShoot2FromCurrentPosition() {

        return lazy(() -> {

            Pose current = follower.pose();

            Path path = Paths.line(
                    current,
                    shoot2
            ).linear(
                    current,
                    shoot2
            );

            return follow(
                    follower,
                    path
            );
        });
    }

    @Override
    public void runOpMode() {

        Scheduler.reset();

        follower =
                Constants.create(hardwareMap);

        follower.setPose(start);
        follower.update();

        limelight =
                hardwareMap.get(
                        Limelight3A.class,
                        "limelight"
                );

        limelight.pipelineSwitch(
                RED_PIPELINE
        );

        limelight.start();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        schedule(autoRoutine());

        while (opModeIsActive()) {

            follower.update();
            Scheduler.execute();

            LLResultTypes.ColorResult target =
                    getBestRedTarget();

            telemetry.addData(
                    "x",
                    follower.pose().x()
            );

            telemetry.addData(
                    "y",
                    follower.pose().y()
            );

            telemetry.addData(
                    "heading",
                    follower.pose().heading()
            );

            if (target != null) {

                telemetry.addData(
                        "Red TX",
                        target.getTargetXDegrees()
                );

                telemetry.addData(
                        "Red area",
                        target.getTargetArea()
                );

                telemetry.addData(
                        "Red bottom Y",
                        getBottomY(target)
                );

            } else {

                telemetry.addData(
                        "Red target",
                        "NONE"
                );
            }

            if (follower.currentPath() != null) {

                telemetry.addData(
                        "Current path distance remaining",
                        follower.distanceToEndpoint()
                );

                telemetry.addData(
                        "Path number",
                        follower.pathIndex()
                );
            }

            telemetry.update();
        }

        limelight.stop();
    }

    public Path shoot1() {

        return Paths.line(
                start,
                shoot1
        ).linear(
                start,
                shoot1
        );
    }

    public Path park() {

        return Paths.curve(
                shoot2,
                parkControl1,
                park
        ).linear(
                shoot2,
                park
        );
    }

    private double clamp(
            double value,
            double min,
            double max) {

        return Math.max(
                min,
                Math.min(max, value)
        );
    }
}