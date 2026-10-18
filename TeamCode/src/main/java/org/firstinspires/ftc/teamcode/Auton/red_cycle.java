
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

@Autonomous(name = "red_cycle", group = "Autonomous")
public class red_cycle extends LinearOpMode {

    private Follower follower;
    private Limelight3A limelight;
    private HiveAim hiveAim;
    private AutoPlan plan;
    private com.qualcomm.hardware.gobilda.GoBildaPinpointDriver pinpoint;

    private final PoseFactory poseFactory =
            PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(58, 9, 90);

    private final Pose readyGarden =
            poseFactory.of(9, 20, 270);

    private final Pose garden =
            poseFactory.of(9, 9, -90);

    private final Pose shoot1 =
            poseFactory.of(31, 29, 46);

    private final Pose flowerReady =
            poseFactory.of(18, 47, 180);

    private final Pose flower =
            poseFactory.of(13.6166, 47, 180);

    private final Pose shoot2 =
            poseFactory.of(31, 29, 46);

    private final Pose pointpark =
            poseFactory.of(13.7021, 91.8842, 256);

    public Command autoRoutine() {
        plan = new AutoPlan(follower, () -> pinpoint.getDeviceStatus()
                == com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.READY);
        return plan
                .add("Ready garden", () -> AutoPlan.follow(follower, this::readyGarden), 8, true, () -> true)
                .add("Garden", () -> AutoPlan.follow(follower, this::garden), 8, true, () -> true)
                .add("Shoot position 1", () -> AutoPlan.follow(follower, this::shoot1), 8, true, () -> true)
                .add("Aim 1", () -> hiveAim.alignAndCalculate("Shoot 1", HiveAim.TAG_PIPELINE),
                        3, false, hiveAim::isAligned)
                .add("Ready flower", this::flowerReadyFromCurrent, 8, true, () -> true)
                .add("Flower", () -> AutoPlan.follow(follower, this::flower), 8, true, () -> true)
                .add("Shoot position 2", () -> AutoPlan.follow(follower, this::shoot2), 8, true, () -> true)
                .add("Aim 2", () -> hiveAim.alignAndCalculate("Shoot 2", HiveAim.TAG_PIPELINE),
                        3, false, hiveAim::isAligned)
                .park(this::parkFromCurrent).command();
    }

    public Path readyGarden() {

        return Paths.line(
                start,
                readyGarden
        ).linear(
                start,
                readyGarden
        );
    }

    public Path garden() {

        return Paths.line(
                readyGarden,
                garden
        ).tangent();
    }

    public Path shoot1() {

        return Paths.line(
                garden,
                shoot1
        ).linear(
                garden,
                shoot1
        );
    }

    public Path flowerReady() {

        return Paths.line(
                shoot1,
                flowerReady
        ).linear(
                shoot1,
                flowerReady
        );
    }

    private Command flowerReadyFromCurrent() {

        return lazy(() -> {

            Pose current = follower.pose();

            Path path = Paths.line(
                    current,
                    flowerReady
            ).linear(
                    current,
                    flowerReady
            );

            return AutoPlan.follow(follower, () -> path);
        });
    }

    public Path flower() {

        return Paths.line(
                flowerReady,
                flower
        ).tangent();
    }

    public Path shoot2() {

        return Paths.line(
                flower,
                shoot2
        ).linear(
                flower,
                shoot2
        );
    }

    public Path park() {

        return Paths.line(
                shoot2,
                pointpark
        ).linear(
                shoot2,
                pointpark
        );
    }

    private Command parkFromCurrent() {

        return lazy(() -> {

            Pose current = follower.pose();

            Path path = Paths.line(
                    current,
                    pointpark
            ).linear(
                    current,
                    pointpark
            );

            return AutoPlan.follow(follower, () -> path);
        });
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

        limelight.pipelineSwitch(
                HiveAim.TAG_PIPELINE
        );

        limelight.start();

        try {

            telemetry.addLine("red_cycle ready");
            telemetry.update();

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            schedule(autoRoutine());

            while (opModeIsActive()) {

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
