package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/** Tag-relative shooting position. Path tracking continues to use Pinpoint. */
public final class LimelightShotPose {
    // Same physical Limelight mount used by SpillCollector, in meters and degrees.
    public static final double HEIGHT = SpillCollector.CAMERA_HEIGHT;
    public static final double PITCH_DEGREES = SpillCollector.CAMERA_PITCH;
    private static final double[] TAG_OFFSETS = {-6.5, -2.75, 2.75, 6.5};
    public String status = "Waiting for Limelight shooting tags";
    public double rangeMeters = Double.NaN;

    public Pose measured(LLResult frame, Pose odometry, Velocity velocity, boolean blue, boolean near,
                         double accelerationX, double accelerationY) {
        rangeMeters = Double.NaN;
        if (!SpillCollector.fresh(frame, VisionRules.SHOT_PIPELINE)
                || !frame.isValid() || frame.getFiducialResults() == null) {
            status = "No fresh Limelight shooting frame";
            return null;
        }
        double age = (frame.getStaleness() + frame.getCaptureLatency()
                + frame.getTargetingLatency() + frame.getParseLatency()) / 1000.0;
        int first = blue ? (near ? 42 : 38) : (near ? 34 : 30);
        double x = 0, y = 0;
        int count = 0;
        for (LLResultTypes.FiducialResult tag : frame.getFiducialResults()) {
            if (!VisionRules.expectedTag(tag.getFiducialId(), first)
                    || tag.getTargetPoseCameraSpace() == null) continue;
            Position camera = tag.getTargetPoseCameraSpace().getPosition();
            if (camera == null) continue;
            camera = camera.toUnit(DistanceUnit.METER);
            Pose estimate = fromCamera(tag.getFiducialId(), camera.x, camera.y, camera.z,
                    age, odometry, velocity, blue, near, accelerationX, accelerationY);
            if (estimate == null) continue;
            x += estimate.x();
            y += estimate.y();
            count++;
        }
        if (count == 0) {
            status = "Limelight target missing, lowered or inconsistent";
            return null;
        }
        Pose result = new Pose(x / count, y / count, odometry.heading());
        double sign = blue ? -1 : 1, end = near ? -1 : 1;
        rangeMeters = Math.hypot(-.32385 * sign - result.x() * .0254,
                .392176 * sign * end - result.y() * .0254);
        status = "Limelight measured " + count + " shooting tags";
        return result;
    }

    /** Limelight camera axes are right, down, forward. Undo pitch before using field yaw. */
    static Pose fromCamera(int id, double right, double down, double forward, double age,
                           Pose odometry, Velocity velocity, boolean blue, boolean near,
                           double accelerationX, double accelerationY) {
        int first = blue ? (near ? 42 : 38) : (near ? 34 : 30);
        if (!VisionRules.expectedTag(id, first)
                || !Double.isFinite(right + down + forward + age + odometry.x()
                    + odometry.y() + odometry.heading() + velocity.vx + velocity.vy + velocity.omega
                    + accelerationX + accelerationY)
                || forward <= .05 || age < 0 || age > VisionRules.MAX_AGE_MS / 1000) return null;
        double pitch = Math.toRadians(PITCH_DEGREES);
        double groundForward = forward * Math.cos(pitch) + down * Math.sin(pitch);
        double tagHeight = HEIGHT + forward * Math.sin(pitch) - down * Math.cos(pitch);
        if (groundForward <= .1 || Math.hypot(groundForward, right) > 4
                || tagHeight < 1.16 || tagHeight > 1.36) return null;
        double sign = blue ? -1 : 1, end = near ? -1 : 1;
        double tagX = sign * (-.32385 - end * TAG_OFFSETS[id - first] * .0254);
        double tagY = sign * end * .327341117;
        double captureHeading = odometry.heading() - velocity.omega * age;
        double c = Math.cos(captureHeading), s = Math.sin(captureHeading);
        double robotX = tagX - groundForward * c - right * s
                + velocity.vx * .0254 * age - .5 * accelerationX * age * age;
        double robotY = tagY - groundForward * s + right * c
                + velocity.vy * .0254 * age - .5 * accelerationY * age * age;
        if (Math.hypot(robotX - odometry.x() * .0254, robotY - odometry.y() * .0254) > .45) return null;
        return new Pose(robotX / .0254, robotY / .0254, odometry.heading());
    }
}
