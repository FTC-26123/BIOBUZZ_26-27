package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58, 9, 90);
    private final Pose readyGarden = poseFactory.of(9, 20, 270);
    private final Pose garden = poseFactory.of(9, 9, -90);
    private final Pose shoot1Start = poseFactory.of(9, 9, 270);
    private final Pose shoot1 = poseFactory.of(31, 29, 46);
    private final Pose flowerReady = poseFactory.of(18, 47, 180);
    private final Pose flower = poseFactory.of(13.6166, 47, 180);
    private final Pose shoot2 = poseFactory.of(31, 29, 46);
    private final Pose point7 = poseFactory.of(13.7021, 91.8842, 256);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, readyGarden()),
                follow(follower, garden()),
                follow(follower, shoot1()),
                follow(follower, flowerReady()),
                follow(follower, flower()),
                follow(follower, shoot2()),
                follow(follower, path7())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path readyGarden() {
        return Paths.line(start, readyGarden).linear(start, readyGarden);
    }

    public Path garden() {
        return Paths.line(readyGarden, garden).tangent();
    }

    public Path shoot1() {
        return Paths.line(shoot1Start, shoot1).linear(shoot1Start, shoot1);
    }

    public Path flowerReady() {
        return Paths.line(shoot1, flowerReady).linear(shoot1, flowerReady);
    }

    public Path flower() {
        return Paths.line(flowerReady, flower).tangent();
    }

    public Path shoot2() {
        return Paths.line(flower, shoot2).linear(flower, shoot2);
    }

    public Path path7() {
        return Paths.line(shoot2, point7).linear(shoot2, point7);
    }
}