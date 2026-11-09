package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Iterator;
import java.util.List;

/** One continuous Pedro curve from the wall start, around the frame corner, into our parking slot. */
public abstract class ParkOnlyAuto extends OpMode {
    /** Keep correcting toward the slot for the whole autonomous period; stop only at the end. */
    private static final double DEADLINE_SECONDS = 29.5;
    /** After this long the arrival window widens so a robot that cannot settle exactly still counts as parked. */
    private static final double LATE_SECONDS = 12;
    private final ElapsedTime timer = new ElapsedTime();
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private Iterator<AutoPaths.Leg> remaining;
    private Pose target;
    private boolean done;

    protected abstract CycleRoute.Start startingPoint();

    /** The designed leg: it lifts off the wall, swings under the frame corner and rises into our slot. */
    static AutoPaths.Leg leg(CycleRoute.Start start) {
        return AutoPaths.curve("Drive around the frame to park", start.blue, start.wallPose(), start.loadingZone(),
            AutoPaths.START_TO_PARK);
    }

    /** Legs actually driven from the measured pose (the designed curve, or the corridor if far off the mark). */
    static List<AutoPaths.Leg> legs(CycleRoute.Start start, Pose actual) {
        return AutoPaths.plan(leg(start), actual, start.blue);
    }

    @Override public void init() {
        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        prepareStart();
    }

    protected final void prepareStart() {
        CycleRoute.Start start = startingPoint();
        Pose wall = start.wallPose();
        follower.setPose(wall);
        follower.update();
        remaining = null;
        done = false;
        telemetry.addData("Place robot (m)", "%.3f, %.3f", wall.x() * .0254, wall.y() * .0254);
        telemetry.addData("Pedro heading (deg)", Math.toDegrees(wall.heading()));
        telemetry.addData("Auto result", "Ready to park");
        telemetry.update();
    }

    @Override public void start() {
        timer.reset();
        remaining = legs(startingPoint(), follower.pose()).iterator();
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
        boolean late = timer.seconds() >= LATE_SECONDS;
        if (distance < (late ? 3.0 : .65) && speed < (late ? 6 : 3)
                && Math.abs(Math.atan2(Math.sin(headingError), Math.cos(headingError))) < Math.toRadians(late ? 6 : 2)) {
            nextWaypoint();
        }
    }

    private void nextWaypoint() {
        if (!remaining.hasNext()) {
            finish("Parked");
            return;
        }
        AutoPaths.Leg leg = remaining.next().from(follower.pose());
        target = leg.end;
        if (!leg.clearOfFrame()) {
            finish("Frame clearance interlock");
            return;
        }
        follower.follow(leg.path());
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
