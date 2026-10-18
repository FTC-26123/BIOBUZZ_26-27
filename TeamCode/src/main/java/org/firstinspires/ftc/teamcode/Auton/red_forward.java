
package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "AutoPathYellow", group = "Autonomous")
public class red_forward extends LinearOpMode {

    private Follower follower;
    private Limelight3A limelight;
    private HiveAim hiveAim;

    private final PoseFactory poseFactory =
            PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(58, 8, 90);

    private final Pose shoot1 =
            poseFactory.of(58, 20, 90);

    private final Pose shoot2 =
            poseFactory.of(58, 20, 90);

    private final Pose park =
            poseFactory.of(12.8411, 92.1374, 256);

    private final Pose parkControl1 =
            poseFactory.of(27.3674, 48.8626, 0);

    private static final int YELLOW_PIPELINE = 0;

    private static final double TURN_KP = 0.025;
    private static final double MAX_TURN = 0.35;
    private static final double SEARCH_TURN = 0.15;

    private static final double FAST_FORWARD = 0.55;
    private static final double SLOW_FORWARD = 0.25;

    private static final double TURN_ONLY_ERROR = 10.0;
    private static final double SLOW_ERROR = 4.0;

    // Temporary example camera calibration.
    private static final double CAMERA_HEIGHT_IN = 12.0;
    private static final double IMAGE_CENTER_Y = 240.0;

    private static final double POLLEN_HEIGHT_IN = 2.8;
    private static final double HEIGHT_TOLERANCE_IN = 0.7;

    private static final double MIN_BOTTOM_OFFSET_PX = 12.0;
    private static final double BALL_CLOSE_BOTTOM_Y = 380.0;

    private static final double SEARCH_TIMEOUT_SECONDS = 6.0;

    private final ElapsedTime searchTimer = new ElapsedTime();

    public Command autoRoutine() {

        return sequential(

                follow(follower, shoot1()),

                hiveAim.alignAndCalculate(
                        "Shoot 1",
                        YELLOW_PIPELINE
                ),

                seekYellowBalls(),

                goToShoot2FromCurrentPosition(),

                hiveAim.alignAndCalculate(
                        "Shoot 2",
                        YELLOW_PIPELINE
                ),

                parkFromCurrent()
        );
    }

    private Command seekYellowBalls() {

        return Command.build()

                .setStart(() -> {

                    searchTimer.reset();

                    limelight.pipelineSwitch(
                            YELLOW_PIPELINE
                    );
                })

                .setExecute(() -> {

                    LLResultTypes.ColorResult target =
                            getBestYellowTarget();

                    if (target == null) {

                        follower.manual(
                                0,
                                0,
                                SEARCH_TURN
                        );

                        return;
                    }

                    double tx = target.getTargetXDegrees();

                    double turn = clamp(
                            TURN_KP * tx,
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

                    if (Double.isFinite(bottomY)
                            && bottomY >= BALL_CLOSE_BOTTOM_Y) {

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

                    if (searchTimer.seconds()
                            >= SEARCH_TIMEOUT_SECONDS) {

                        return true;
                    }

                    LLResultTypes.ColorResult target =
                            getBestYellowTarget();

                    return target != null
                            && getBottomY(target)
                            >= BALL_CLOSE_BOTTOM_Y;
                })

                .setEnd(endCondition -> {

                    follower.manual(0, 0, 0);
                });
    }

    private LLResultTypes.ColorResult getBestYellowTarget() {

        LLResult result = limelight.getLatestResult();

        if (result == null
                || !org.firstinspires.ftc.teamcode.subsystems.Limelight.isFresh(result)
                || result.getPipelineIndex() != YELLOW_PIPELINE) {

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

            if (!isPollen(target)) {
                continue;
            }

            double area = target.getTargetArea();

            if (area > biggestArea) {

                biggestArea = area;
                bestTarget = target;
            }
        }

        return bestTarget;
    }

    private boolean isPollen(
            LLResultTypes.ColorResult target) {

        double estimatedHeight =
                estimateRealHeight(target);

        if (!Double.isFinite(estimatedHeight)) {
            return false;
        }

        return Math.abs(
                estimatedHeight - POLLEN_HEIGHT_IN
        ) <= HEIGHT_TOLERANCE_IN;
    }

    private double estimateRealHeight(
            LLResultTypes.ColorResult target) {

        double[] bounds = getVerticalBounds(target);

        if (bounds == null) {
            return Double.NaN;
        }

        double topY = bounds[0];
        double bottomY = bounds[1];

        double pixelHeight = bottomY - topY;
        double bottomOffset = bottomY - IMAGE_CENTER_Y;

        if (pixelHeight <= 0
                || bottomOffset <= MIN_BOTTOM_OFFSET_PX) {

            return Double.NaN;
        }

        return CAMERA_HEIGHT_IN
                * pixelHeight
                / bottomOffset;
    }

    private double[] getVerticalBounds(
            LLResultTypes.ColorResult target) {

        List<List<Double>> corners =
                target.getTargetCorners();

        if (corners == null || corners.isEmpty()) {
            return null;
        }

        double topY = Double.POSITIVE_INFINITY;
        double bottomY = Double.NEGATIVE_INFINITY;

        for (List<Double> corner : corners) {

            if (corner == null || corner.size() < 2) {
                continue;
            }

            double y = corner.get(1);

            if (!Double.isFinite(y)) {
                continue;
            }

            topY = Math.min(topY, y);
            bottomY = Math.max(bottomY, y);
        }

        if (!Double.isFinite(topY)
                || !Double.isFinite(bottomY)
                || bottomY <= topY) {

            return null;
        }

        return new double[]{
                topY,
                bottomY
        };
    }

    private double getBottomY(
            LLResultTypes.ColorResult target) {

        double[] bounds = getVerticalBounds(target);

        return bounds == null
                ? Double.NaN
                : bounds[1];
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

            return follow(follower, path);
        });
    }

    private Command parkFromCurrent() {

        return lazy(() -> {

            Pose current = follower.pose();

            Path path = Paths.curve(
                    current,
                    parkControl1,
                    park
            ).linear(
                    current,
                    park
            );

            return follow(follower, path);
        });
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

    private double clamp(
            double value,
            double minimum,
            double maximum) {

        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }

    @Override
    public void runOpMode() {

        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        follower.setPose(start);
        follower.update();

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        hiveAim = new HiveAim(
                follower,
                limelight
        );

        limelight.pipelineSwitch(
                YELLOW_PIPELINE
        );

        limelight.start();

        try {

            telemetry.addLine(
                    "Yellow autonomous ready"
            );

            telemetry.update();

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            schedule(autoRoutine());

            while (opModeIsActive()) {

                follower.update();
                Scheduler.execute();

                telemetry.addData(
                        "x",
                        follower.pose().x()
                );

                telemetry.addData(
                        "y",
                        follower.pose().y()
                );

                telemetry.addData(
                        "heading (deg)",
                        Math.toDegrees(
                                follower.pose().heading()
                        )
                );

                hiveAim.addTelemetry(telemetry);

                LLResultTypes.ColorResult target =
                        getBestYellowTarget();

                if (target != null) {

                    telemetry.addData(
                            "Yellow TX",
                            target.getTargetXDegrees()
                    );

                    telemetry.addData(
                            "Yellow area",
                            target.getTargetArea()
                    );

                    telemetry.addData(
                            "Estimated height (in)",
                            estimateRealHeight(target)
                    );

                    telemetry.addData(
                            "Bottom Y",
                            getBottomY(target)
                    );

                } else {

                    telemetry.addData(
                            "Yellow target",
                            "NONE"
                    );
                }

                if (follower.currentPath() != null) {

                    telemetry.addData(
                            "Distance remaining",
                            follower.distanceToEndpoint()
                    );

                    telemetry.addData(
                            "Path number",
                            follower.pathIndex()
                    );
                }

                telemetry.update();
                idle();
            }

        } finally {

            Scheduler.reset();

            follower.stop();
            follower.drivetrain.stop();

            limelight.stop();
        }
    }
}
