package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Auton.cycles.LimelightMount;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.ShotConfig;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * LIMELIGHT CALIBRATION: works out the camera's tilt (Pitch deg) and how far the lens is in front of the robot centre
 * (Forward in) from two taped distances, so neither has to be measured. Height and left/right are the measured values
 * in LimelightMount (12 3/8 in, centred).
 *
 * <pre>
 * 1. Cell UP. Tape a line on the floor straight out from the point under the MIDDLE of the cell's tag sticker.
 *    Mark 30 in and 78 in along it (measured to the robot CENTRE).
 * 2. Robot centre on the 30 in mark, square to the cell, all 4 tags in view. Press A (captures about 1 s of frames).
 * 3. Same on the 78 in mark. Press Y.
 * 4. Read the result. START saves it to Robot Settings > Camera. B = start over.
 *    Used other marks? Dpad Up/Down changes the near distance, Dpad Right/Left the far one, before capturing.
 * </pre>
 * The robot does not move. Nothing is saved until START.
 */
@TeleOp(name = "Limelight Calibration (pitch + forward)", group = "Setup")
public class TeleOp_LimelightCalibration extends OpMode {
    private static final int FRAMES = 30;

    private final Limelight limelight = new Limelight();
    private boolean cameraOk;
    private double nearIn = 30, farIn = 78;
    /** Captured camera-space tag-row middles {x right, y down, z forward}, metres; null until captured. */
    private double[] near, far;
    private final List<double[]> capturing = new ArrayList<>();
    private int capturingFor = 0;   // 0 = not capturing, 1 = near, 2 = far
    private double pitchDeg = Double.NaN, forwardIn = Double.NaN, heightNearIn, heightFarIn;
    private String message = "";

    @Override
    public void init() {
        RobotSettings.ensureLoaded();
        cameraOk = limelight.initSafely(hardwareMap, ShotConfig.APRILTAG_PIPELINE);
        if (cameraOk) limelight.start();
    }

    @Override
    public void init_loop() { loop(); }

    @Override
    public void loop() {
        if (!cameraOk) {
            telemetry.addLine("Limelight NOT FOUND: check the cable and the \"limelight\" configuration name.");
            telemetry.update();
            return;
        }
        limelight.update();
        double[] now = tagRowMiddle(limelight.getLastResult());

        if (capturingFor == 0) {
            if (gamepad1.dpadUpWasPressed()) nearIn += 1;
            if (gamepad1.dpadDownWasPressed()) nearIn -= 1;
            if (gamepad1.dpadRightWasPressed()) farIn += 1;
            if (gamepad1.dpadLeftWasPressed()) farIn -= 1;
            if (gamepad1.aWasPressed()) startCapture(1);
            if (gamepad1.yWasPressed()) startCapture(2);
            if (gamepad1.bWasPressed()) { near = far = null; pitchDeg = forwardIn = Double.NaN; message = "Cleared."; }
            if (gamepad1.startWasPressed()) save();
        } else {
            if (now != null) capturing.add(now);
            if (capturing.size() >= FRAMES) finishCapture();
        }

        telemetry.addLine("=== LIMELIGHT CALIBRATION ===");
        telemetry.addData("Tags", now == null ? "NOT ALL 4 IN VIEW - point straight at the middle of the cell's tags"
                : String.format(Locale.US, "all 4 seen, %.1f in straight-line from the lens", norm(now) / .0254));
        telemetry.addData("A = near capture", "%.0f in  %s  (Dpad Up/Down)", nearIn, near == null ? "-" : "CAPTURED");
        telemetry.addData("Y = far capture", "%.0f in  %s  (Dpad Right/Left)", farIn, far == null ? "-" : "CAPTURED");
        if (capturingFor != 0) telemetry.addLine("Capturing... keep still (" + capturing.size() + "/" + FRAMES + ")");
        if (!Double.isNaN(pitchDeg)) {
            telemetry.addData("RESULT Pitch deg", "%.2f   (now %.2f)", pitchDeg, LimelightMount.PITCH_DEGREES);
            telemetry.addData("RESULT Forward in", "%.2f   (now %.2f)", forwardIn, LimelightMount.FORWARD_METERS / .0254);
            telemetry.addData("Tag height check", "%.1f in near, %.1f in far %s", heightNearIn, heightFarIn,
                    Math.abs(heightNearIn - heightFarIn) > 1.0
                            ? "- DO NOT AGREE: redo (robot square? on the marks? cell moved?)" : "- agree");
            telemetry.addLine("START = save to Robot Settings > Camera.  B = start over.");
        }
        telemetry.addData("Height / Right (measured)", "%.3f in / %.2f in", LimelightMount.MEASURED_HEIGHT_IN,
                LimelightMount.MEASURED_RIGHT_IN);
        if (!message.isEmpty()) telemetry.addLine(message);
        telemetry.update();
    }

    private void startCapture(int which) {
        capturing.clear();
        capturingFor = which;
        message = "";
    }

    private void finishCapture() {
        double[] mean = new double[3];
        for (double[] p : capturing) for (int k = 0; k < 3; k++) mean[k] += p[k] / capturing.size();
        if (capturingFor == 1) near = mean; else far = mean;
        capturingFor = 0;
        capturing.clear();
        if (near != null && far != null) solve();
    }

    /**
     * Same geometry as TargetTracker: the tag row's distance in front of the lens, level with the floor, is
     * z cos(pitch) + y sin(pitch). Forward is the same at both marks, so the right pitch is the one that makes the
     * two captures differ by exactly (far - near). Forward then makes both distances come out right (average).
     */
    private void solve() {
        double right = LimelightMount.MEASURED_RIGHT_IN * .0254, gap = (farIn - nearIn) * .0254;
        double best = Double.NaN, bestErr = Double.POSITIVE_INFINITY;
        for (double deg = -10; deg <= 80; deg += 0.01) {
            double err = Math.abs((level(far, deg) - level(near, deg)) - gap);
            if (err < bestErr) { bestErr = err; best = deg; }
        }
        double fwdNear = Math.sqrt(Math.max(0, sq(nearIn * .0254) - sq(near[0] + right))) - level(near, best);
        double fwdFar = Math.sqrt(Math.max(0, sq(farIn * .0254) - sq(far[0] + right))) - level(far, best);
        pitchDeg = best;
        forwardIn = (fwdNear + fwdFar) / 2 / .0254;
        double p = Math.toRadians(best), h = LimelightMount.MEASURED_HEIGHT_IN * .0254;
        heightNearIn = (h + near[2] * Math.sin(p) - near[1] * Math.cos(p)) / .0254;
        heightFarIn = (h + far[2] * Math.sin(p) - far[1] * Math.cos(p)) / .0254;
        message = bestErr > .01 ? "Could not match the two distances: check the marks and that the robot is square." : "";
    }

    private static double level(double[] cam, double pitchDeg) {
        double p = Math.toRadians(pitchDeg);
        return cam[2] * Math.cos(p) + cam[1] * Math.sin(p);
    }

    private void save() {
        if (Double.isNaN(pitchDeg)) { message = "Nothing to save: capture near (A) and far (Y) first."; return; }
        LimelightMount.PITCH_DEGREES = pitchDeg;
        LimelightMount.FORWARD_METERS = forwardIn * .0254;
        LimelightMount.HEIGHT_METERS = LimelightMount.MEASURED_HEIGHT_IN * .0254;
        LimelightMount.RIGHT_METERS = LimelightMount.MEASURED_RIGHT_IN * .0254;
        message = RobotSettings.save() ? "SAVED. Check in Shot Table Builder: the tags reading should match the tape."
                : "SAVE FAILED - enter the numbers in Robot Settings > Camera by hand.";
    }

    /** Middle of a cell's tag row in camera space {x right, y down, z forward}, metres, only when all 4 are seen. */
    private static double[] tagRowMiddle(LLResult frame) {
        if (frame == null || !frame.isValid()) return null;
        List<LLResultTypes.FiducialResult> tags = frame.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;
        int lowest = Math.min(ShotConfig.RED_FIRST_TAG, ShotConfig.BLUE_FIRST_TAG);
        int cells = 2 * ShotConfig.BASKETS_PER_ALLIANCE;
        double[][] sum = new double[cells][3];
        int[] count = new int[cells];
        for (LLResultTypes.FiducialResult t : tags) {
            int cell = (t.getFiducialId() - lowest) / ShotConfig.TAGS_PER_BASKET;
            if (t.getFiducialId() < lowest || cell >= cells) continue;
            Pose3D pose = t.getTargetPoseCameraSpace();
            if (pose == null || pose.getPosition() == null) continue;
            Position p = pose.getPosition().toUnit(DistanceUnit.METER);
            if (Double.isNaN(p.x + p.y + p.z) || p.z <= 0.05) continue;
            sum[cell][0] += p.x;
            sum[cell][1] += p.y;
            sum[cell][2] += p.z;
            count[cell]++;
        }
        // The average of the tags is the middle of the row only when all four are seen (they are symmetric).
        for (int c = 0; c < cells; c++) {
            if (count[c] == ShotConfig.TAGS_PER_BASKET)
                return new double[] {sum[c][0] / count[c], sum[c][1] / count[c], sum[c][2] / count[c]};
        }
        return null;
    }

    private static double sq(double v) { return v * v; }

    private static double norm(double[] v) { return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

    @Override
    public void stop() { limelight.stop(); }
}
