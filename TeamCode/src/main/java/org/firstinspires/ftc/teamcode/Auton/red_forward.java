
package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;


import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.lazy;

@Autonomous(name = "AutoPathYellow", group = "Autonomous")
public class red_forward extends LinearOpMode {

    private Follower follower;
    private Limelight3A limelight;
    private HiveAim hiveAim;
    private YellowBallSeeker seeker;
    private AutoPlan plan;
    private com.qualcomm.hardware.gobilda.GoBildaPinpointDriver pinpoint;

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

    public Command autoRoutine() {
        plan = new AutoPlan(follower, () -> pinpoint.getDeviceStatus()
                == com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.READY);
        return plan
                .add("Shoot position 1", () -> AutoPlan.follow(follower, this::shoot1), 8, true, () -> true)
                .add("Aim 1", () -> hiveAim.alignAndCalculate("Shoot 1", YellowBallSeeker.PIPELINE),
                        3, false, hiveAim::isAligned)
                .add("Seek yellow", seeker::command, 6.5, false, seeker::acquired)
                .add("Shoot position 2", this::goToShoot2FromCurrentPosition, 8, true, () -> true)
                .add("Aim 2", () -> hiveAim.alignAndCalculate("Shoot 2", YellowBallSeeker.PIPELINE),
                        3, false, hiveAim::isAligned)
                .park(this::parkFromCurrent).command();
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

            return AutoPlan.follow(follower, () -> path);
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

            return AutoPlan.follow(follower, () -> path);
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

    @Override
    public void runOpMode() {

        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        pinpoint = hardwareMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class, "pinpoint");

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

        seeker = new YellowBallSeeker(follower, limelight);
        limelight.pipelineSwitch(YellowBallSeeker.PIPELINE);

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

                seeker.sample();
                Scheduler.execute();
                follower.update();

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

                seeker.addTelemetry(telemetry);
                plan.addTelemetry(telemetry);

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
