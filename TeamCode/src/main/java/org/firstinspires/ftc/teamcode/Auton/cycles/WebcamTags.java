package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

/** USB webcam checks tag identity only; Pinpoint still supplies the shooting pose. */
public final class WebcamTags {
    public static final String NAME = "Webcam 1";
    private final AprilTagProcessor processor;
    private final VisionPortal portal;
    private boolean closed;
    public String status = "Waiting for webcam tags";

    public WebcamTags(HardwareMap hardwareMap) {
        processor = AprilTagProcessor.easyCreateWithDefaults();
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, NAME), processor);
    }
    public boolean ready(int first) {
        if (closed) { status = "Webcam closed - holding fire"; return false; }
        long now = System.nanoTime();
        boolean fresh = false;
        for (AprilTagDetection detection : processor.getDetections()) {
            if (!(detection instanceof AprilTagSingleDetection)) continue;
            AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
            if (!VisionRules.freshTag(now, tag.frameAcquisitionNanoTime)) continue;
            fresh = true;
            if (VisionRules.expectedTag(tag.id, first)) {
                status = "Webcam verified cell " + first;
                return true;
            }
        }
        status = fresh ? "Webcam wrong cell - holding fire" : "No fresh webcam tag - holding fire";
        return false;
    }
    public void close() {
        if (!closed) { closed = true; portal.close(); }
    }
}
