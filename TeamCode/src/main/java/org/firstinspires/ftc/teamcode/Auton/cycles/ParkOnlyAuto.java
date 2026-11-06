package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/** Clear the start wall, drive around the frame, and stop in the loading zone. */
public abstract class ParkOnlyAuto extends OpMode {
    private static final double DEADLINE_SECONDS = 12;
    private final ElapsedTime timer = new ElapsedTime();
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private Iterator<Pose> remaining;
    private Pose target;
    private boolean done;

    protected abstract CycleRoute.Start startingPoint();

    static List<Pose> waypoints(CycleRoute.Start start) {
        Pose wall = start.wallPose();
        double inward = (start.blue ? -1 : 1) * .134259 / .0254;
        Pose clearWall = new Pose(wall.x(), wall.y() + inward, wall.heading());
        List<Pose> path = new ArrayList<>();
        path.add(clearWall);
        path.addAll(FrameClearance.path(clearWall, start.loadingZone(), start.blue));
        return path;
    }

    @Override public void init() {
        CycleRoute.Start start = startingPoint();
        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        Pose wall = start.wallPose();
        follower.setPose(wall);
        follower.update();
        remaining = waypoints(start).iterator();
        telemetry.addData("Place robot (m)", "%.3f, %.3f", wall.x() * .0254, wall.y() * .0254);
        telemetry.addData("Pedro heading (deg)", Math.toDegrees(wall.heading()));
        telemetry.addData("Auto result", "Ready to park");
        telemetry.update();
    }

    @Override public void start() {
        timer.reset();
        nextWaypoint();
    }

    @Override public void loop() {
        if (done) return;
        if (timer.seconds() >= DEADLINE_SECONDS) {
            finish("Parking timeout");
            return;
        }
        follower.update();
        Pose pose = follower.pose();
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                || !Double.isFinite(pose.x() + pose.y() + pose.heading())) {
            finish("Localization fault");
            return;
        }
        if (!FrameClearance.clear(pose.x() * .0254, pose.y() * .0254)) {
            finish("Frame clearance interlock");
            return;
        }
        double distance = Math.hypot(pose.x() - target.x(), pose.y() - target.y());
        if (distance < 2 && !follower.holding()) follower.hold(target);
        double headingError = pose.heading() - target.heading();
        double speed = Math.hypot(follower.velocity().vx, follower.velocity().vy);
        if (distance < .65 && speed < 3
                && Math.abs(Math.atan2(Math.sin(headingError), Math.cos(headingError))) < Math.toRadians(2)) {
            nextWaypoint();
        }
    }

    private void nextWaypoint() {
        if (!remaining.hasNext()) {
            finish("Parked");
            return;
        }
        target = remaining.next();
        Pose from = follower.pose();
        if (!FrameClearance.segment(from.x() * .0254, from.y() * .0254,
                target.x() * .0254, target.y() * .0254)) {
            finish("Frame clearance interlock");
            return;
        }
        follower.follow(Paths.line(from, target).linear(from, target));
        telemetry.addData("Auto result", "Driving to park");
        telemetry.update();
    }

    private void finish(String result) {
        done = true;
        stop();
        telemetry.addData("Auto result", result);
        telemetry.addData("Auto elapsed", timer.seconds());
        telemetry.update();
    }

    @Override public void stop() {
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop();
        }
    }
}
