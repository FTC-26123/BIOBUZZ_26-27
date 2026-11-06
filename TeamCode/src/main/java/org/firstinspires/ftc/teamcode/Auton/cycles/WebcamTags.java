package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

/** USB webcam checks tag identity only; Pinpoint still supplies the shooting pose. */
public final class WebcamTags {
    public static final String NAME = "Webcam 1";
    private final AprilTagProcessor processor;
    private final VisionPortal portal;
    private static final double CAMERA_HEIGHT_METERS = .305;
    private static final double CAMERA_PITCH = Math.toRadians(10);
    private static final double[] FAR_TAG_OFFSETS_INCHES = {-6.5, -2.75, 2.75, 6.5};
    private boolean closed;
    public String status = "Waiting for webcam tags";

    public WebcamTags(HardwareMap hardwareMap) {
        AprilTagLibrary.Builder library = new AprilTagLibrary.Builder();
        for (int id = 30; id <= 45; id++) {
            library.addTag(id, "Hive " + id, 3.25, DistanceUnit.INCH);
        }
        processor = new AprilTagProcessor.Builder().setTagLibrary(library.build()).build();
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, NAME), processor);
    }
    public boolean ready(int first) {
        return ready(first, null, false);
    }

    /** Far-board IDs can be visible underneath the hive before the tip. */
    public boolean readyRaisedFar(int first, Pose robot, boolean blue) {
        return ready(first, robot, blue);
    }

    private boolean ready(int first, Pose raisedFrom, boolean blue) {
        if (closed) { status = "Webcam closed - holding fire"; return false; }
        long now = System.nanoTime();
        boolean fresh = false;
        boolean expected = false;
        for (AprilTagDetection detection : processor.getDetections()) {
            if (!(detection instanceof AprilTagSingleDetection)) continue;
            AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
            if (!VisionRules.freshTag(now, tag.frameAcquisitionNanoTime)) continue;
            fresh = true;
            if (VisionRules.expectedTag(tag.id, first)) {
                expected = true;
                if (raisedFrom != null && !raised(tag, first, raisedFrom, blue)) continue;
                status = "Webcam verified cell " + first;
                return true;
            }
        }
        if (expected) {
            status = "Far cell still down or pose unavailable - holding fire";
            return false;
        }
        status = fresh ? "Webcam wrong cell - holding fire" : "No fresh webcam tag - holding fire";
        return false;
    }
    private boolean raised(AprilTagSingleDetection tag, int first, Pose robot, boolean blue) {
        if (tag.ftcPose == null || tag.ftcPose.y <= 0) return false;
        // Undo camera pitch, then use the ray slope and odometry range to estimate
        // height. The slope is independent of tag range scale and output units.
        double forward = tag.ftcPose.y * Math.cos(CAMERA_PITCH) - tag.ftcPose.z * Math.sin(CAMERA_PITCH);
        double up = tag.ftcPose.y * Math.sin(CAMERA_PITCH) + tag.ftcPose.z * Math.cos(CAMERA_PITCH);
        double sign = blue ? -1 : 1;
        double tagX = sign * (-.32385 - FAR_TAG_OFFSETS_INCHES[tag.id - first] * .0254);
        double tagY = sign * .327341117;
        double horizontalRange = Math.hypot(tagX - robot.x() * .0254, tagY - robot.y() * .0254);
        double height = CAMERA_HEIGHT_METERS + horizontalRange * up / Math.hypot(tag.ftcPose.x, forward);
        // The lower board is around 0.91 m, the raised board around 1.26 m.
        return height >= 1.22 && height <= 1.34;
    }

    public void close() {
        if (!closed) { closed = true; portal.close(); }
    }
}
