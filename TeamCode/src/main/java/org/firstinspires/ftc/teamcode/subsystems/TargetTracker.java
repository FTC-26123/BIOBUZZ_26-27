package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Auton.AutoConfig;
import org.firstinspires.ftc.teamcode.Auton.cycles.LimelightMount;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Finds our alliance's upward CELL in a Limelight frame and works out the exact 3D AIM POINT inside its opening,
 * from any number of visible tags (one is enough). Uses a standard AprilTag pipeline (one result per tag).
 *
 * <p>Geometry, from the game manual (sections 9.6 and 9.9):
 * <ul>
 *   <li>Each CELL has one sticker of 4 tags on its flat bottom face. Along the sticker, tag centres are at
 *       -6.5, -2.75, +2.75, +6.5 in from the cluster centre, ids increasing left to right as you face them.
 *       The cluster centre is on the CELL's left-right centreline, so the aim point is always dead centre.</li>
 *   <li>The HIVE rests tilted {@link ShotConfig#HIVE_TILT_DEG} (30) degrees. For the upward CELL the opening
 *       faces out and up; the cluster centre is {@link ShotConfig#CLUSTER_TO_OPENING_IN} (7.19) in back from
 *       the opening along the bottom face, and the opening is {@link ShotConfig#OPENING_HEIGHT_IN} (14) in
 *       from its bottom lip to its top.</li>
 * </ul>
 * The aim point = cluster centre, moved out to the opening, then up the opening by
 * {@link ShotConfig#AIM_HEIGHT_FRACTION} (0 = bottom lip, 1 = top), then into the CELL by
 * {@link ShotConfig#AIM_DEPTH_IN}. Only the height fraction is meant to be tuned (it depends on our shooter).
 *
 * <p>Robot frame used here: F = forward, R = right, U = up from the floor, all inches, origin = robot centre
 * on the floor. Limelight camera axes: x = right, y = down, z = out of the lens.
 */
public final class TargetTracker {
    private static final int HISTORY = 5;
    /** Tag centres along the sticker from the cluster centre, inches, in id order (game manual Fig. 9-15). */
    public static final double[] TAG_OFFSETS_IN = {-6.5, -2.75, 2.75, 6.5};

    private final double[] history = new double[HISTORY];
    private int historyCount = 0, historyNext = 0;

    private boolean visible = false, cellUp = false;
    private double aimErrorDeg = 0;
    private double distanceInches = Double.NaN, aimHeightInches = Double.NaN, clusterDistanceInches = Double.NaN;
    private double cameraTxDeg = Double.NaN, cameraTyDeg = Double.NaN;
    private double lastSeenSec = Double.NEGATIVE_INFINITY;
    private int tagCount = 0, basketFirstTag = -1;
    private double tagHeightInches = Double.NaN;
    private int[] tagIds = new int[0];
    private double[] tagForwardMeters = new double[0], tagRightMeters = new double[0];

    /** One visible tag of ours, in the robot frame. */
    private static final class Tag {
        int id;
        double offset, txDeg;
        double[] pos;   // {F, R, U} inches
    }

    /**
     * Call once per loop.
     *
     * @param frame  latest Limelight result (may be null)
     * @param blue   true = blue alliance tags (38-45), false = red (30-37). Other tags are thrown away first.
     * @param nowSec any running clock in seconds
     */
    public void update(LLResult frame, boolean blue, double nowSec) {
        visible = false;
        tagCount = 0;
        tagIds = new int[0];
        tagForwardMeters = new double[0];
        tagRightMeters = new double[0];
        cameraTxDeg = cameraTyDeg = Double.NaN;

        List<LLResultTypes.FiducialResult> raw = null;
        if (frame != null && frame.isValid() && frame.getStaleness() <= ShotConfig.MAX_FRAME_AGE_MS
                && (InSimulator.YES || frame.getPipelineIndex() == ShotConfig.APRILTAG_PIPELINE)) {
            raw = frame.getFiducialResults();
        }
        // 1) Keep only our alliance's tags, grouped by CELL (4 ids each).
        int first = blue ? ShotConfig.BLUE_FIRST_TAG : ShotConfig.RED_FIRST_TAG;
        int per = ShotConfig.TAGS_PER_BASKET, cells = ShotConfig.BASKETS_PER_ALLIANCE;
        List<List<Tag>> byCell = new ArrayList<>();
        for (int c = 0; c < cells; c++) byCell.add(new ArrayList<>());
        if (raw != null) {
            for (LLResultTypes.FiducialResult r : raw) {
                int index = r.getFiducialId() - first;
                if (index < 0 || index >= cells * per) continue;
                double[] pos = robotFrameInches(r.getTargetPoseCameraSpace());
                if (pos == null) continue;
                Tag t = new Tag();
                t.id = r.getFiducialId();
                t.offset = TAG_OFFSETS_IN[Math.min(TAG_OFFSETS_IN.length - 1, index % per)];
                t.txDeg = r.getTargetXDegrees();
                t.pos = pos;
                byCell.get(index / per).add(t);
            }
        }

        // 2) Pick the CELL: an UPWARD one (cluster high) first, then the one nearest the middle of the picture.
        int best = -1;
        boolean bestUp = false;
        double bestTx = 0;
        for (int c = 0; c < cells; c++) {
            List<Tag> tags = byCell.get(c);
            if (tags.isEmpty()) continue;
            double height = 0, tx = 0;
            for (Tag t : tags) { height += t.pos[2]; tx += t.txDeg; }
            height /= tags.size();
            tx = Math.abs(tx / tags.size());
            boolean up = height >= AutoConfig.BASKET_UP_TAG_HEIGHT_IN;
            if (best < 0 || (up && !bestUp) || (up == bestUp && tx < bestTx)) {
                best = c;
                bestUp = up;
                bestTx = tx;
            }
        }

        if (best >= 0) {
            List<Tag> tags = byCell.get(best);
            visible = true;
            cellUp = bestUp;
            tagCount = tags.size();
            if (basketFirstTag != first + best * per) historyCount = historyNext = 0;   // switched CELL
            basketFirstTag = first + best * per;
            solve(tags);
            lastSeenSec = nowSec;

            tagIds = new int[tags.size()];
            tagForwardMeters = new double[tags.size()];
            tagRightMeters = new double[tags.size()];
            for (int i = 0; i < tags.size(); i++) {
                tagIds[i] = tags.get(i).id;
                tagForwardMeters[i] = tags.get(i).pos[0] * 0.0254;
                tagRightMeters[i] = tags.get(i).pos[1] * 0.0254;
            }
        }
        if (!visible && nowSec - lastSeenSec > ShotConfig.DISTANCE_HOLD_SEC) {
            historyCount = historyNext = 0;
            distanceInches = aimHeightInches = clusterDistanceInches = Double.NaN;
        }
    }

    /** Cluster centre and aim point from the visible tags of one CELL. */
    private void solve(List<Tag> tags) {
        int n = tags.size();
        double[] mean = new double[3];
        double meanOffset = 0;
        for (Tag t : tags) {
            for (int k = 0; k < 3; k++) mean[k] += t.pos[k] / n;
            meanOffset += t.offset / n;
        }
        double[] cam = {LimelightMount.FORWARD_METERS / 0.0254, LimelightMount.RIGHT_METERS / 0.0254,
                LimelightMount.HEIGHT_METERS / 0.0254};

        // 3) The sticker's left-right direction (always level). Default: square to our line of sight, which is
        // close enough for one tag (the error is mostly along the line of sight). With two or more tags it is
        // fitted through the tags themselves.
        double losF = mean[0] - cam[0], losR = mean[1] - cam[1], len = Math.hypot(losF, losR);
        double lF = -losR / len, lR = losF / len;
        if (n >= 2) {
            double fF = 0, fR = 0;
            for (Tag t : tags) {
                fF += (t.offset - meanOffset) * (t.pos[0] - mean[0]);
                fR += (t.offset - meanOffset) * (t.pos[1] - mean[1]);
            }
            double fl = Math.hypot(fF, fR);
            if (fl > 1e-6 && (fF * lF + fR * lR) / fl > 0.5) {   // trust the fit unless it is far off
                lF = fF / fl;
                lR = fR / fl;
            }
        }

        // 4) Cluster centre: each tag minus its own known offset along the sticker, averaged.
        double[] centre = new double[3];
        for (Tag t : tags) {
            centre[0] += (t.pos[0] - t.offset * lF) / n;
            centre[1] += (t.pos[1] - t.offset * lR) / n;
            centre[2] += t.pos[2] / n;
        }
        tagHeightInches = centre[2];
        clusterDistanceInches = Math.hypot(centre[0], centre[1]);

        // 5) Aim point in the opening (upward CELL only; a lowered CELL is aimed at its cluster centre).
        double[] aim = centre.clone();
        if (cellUp) {
            // Level direction out of the HIVE toward the robot, square to the sticker.
            double hF = lR, hR = -lF;
            if (hF * (cam[0] - centre[0]) + hR * (cam[1] - centre[1]) < 0) { hF = -hF; hR = -hR; }
            double tilt = Math.toRadians(ShotConfig.HIVE_TILT_DEG);
            double c = Math.cos(tilt), s = Math.sin(tilt);
            // d: from the cluster out to the opening (out and up). a: up the opening, bottom lip to top (in and up).
            double out = ShotConfig.CLUSTER_TO_OPENING_IN - ShotConfig.AIM_DEPTH_IN;
            double up = ShotConfig.AIM_HEIGHT_FRACTION * ShotConfig.OPENING_HEIGHT_IN;
            double alongH = out * c - up * s, alongU = out * s + up * c;
            aim[0] += alongH * hF;
            aim[1] += alongH * hR;
            aim[2] += alongU;
        }

        // 6) Turn angle from the robot centre, distance, height, and the camera's own tx/ty to the aim point.
        aimErrorDeg = Math.toDegrees(Math.atan2(aim[1], aim[0]));
        aimHeightInches = aim[2];
        history[historyNext] = Math.hypot(aim[0], aim[1]);
        historyNext = (historyNext + 1) % HISTORY;
        historyCount = Math.min(HISTORY, historyCount + 1);
        distanceInches = median();

        double pitch = Math.toRadians(LimelightMount.PITCH_DEGREES);
        double dF = aim[0] - cam[0], dR = aim[1] - cam[1], dU = aim[2] - cam[2];
        double z = dF * Math.cos(pitch) + dU * Math.sin(pitch), y = dF * Math.sin(pitch) - dU * Math.cos(pitch);
        if (z > 0) {
            cameraTxDeg = Math.toDegrees(Math.atan2(dR, z));
            cameraTyDeg = Math.toDegrees(Math.atan2(-y, z));
        }
    }

    /**
     * One tag's position in the robot frame, inches {forward, right, up from the floor}. The camera looks up by
     * the mount pitch: forward = z*cos(pitch) + y*sin(pitch), up = z*sin(pitch) - y*cos(pitch).
     */
    private static double[] robotFrameInches(Pose3D tagInCamera) {
        if (tagInCamera == null) return null;
        Position p = tagInCamera.getPosition();
        if (p == null) return null;
        p = p.toUnit(DistanceUnit.METER);
        if (Double.isNaN(p.x + p.y + p.z) || p.z <= 0.05) return null;
        double pitch = Math.toRadians(LimelightMount.PITCH_DEGREES);
        double forward = p.z * Math.cos(pitch) + p.y * Math.sin(pitch);
        if (forward <= 0.1) return null;
        double ahead = forward + LimelightMount.FORWARD_METERS, right = p.x + LimelightMount.RIGHT_METERS;
        if (Math.hypot(ahead, right) > 5) return null;
        double up = LimelightMount.HEIGHT_METERS + p.z * Math.sin(pitch) - p.y * Math.cos(pitch);
        return new double[] {ahead / 0.0254, right / 0.0254, up / 0.0254};
    }

    private double median() {
        double[] sorted = Arrays.copyOf(history, historyCount);
        Arrays.sort(sorted);
        return sorted[historyCount / 2];
    }

    /** @return true if one of our CELLs' tags is in the current frame. */
    public boolean isVisible() { return visible; }

    /** @return true if the tracked CELL is the upward one (the one to shoot into). */
    public boolean isCellUp() { return visible && cellUp; }

    /** Degrees the robot must turn to face the aim point; positive = to the right. Only meaningful while visible. */
    public double getAimErrorDeg() { return aimErrorDeg; }

    /** Floor distance in inches from the robot centre to the AIM POINT (median of the last frames), or NaN. */
    public double getDistanceInches() { return distanceInches; }

    /** Height of the aim point above the floor, inches (last known), or NaN. */
    public double getAimHeightInches() { return aimHeightInches; }

    /** Floor distance from the robot centre to the tag cluster centre (tape-measurable), inches, or NaN. */
    public double getClusterDistanceInches() { return clusterDistanceInches; }

    /** The camera's own horizontal / vertical angle to the aim point (degrees), NaN when not visible. */
    public double getCameraTxDeg() { return cameraTxDeg; }
    public double getCameraTyDeg() { return cameraTyDeg; }

    /** @return true if a distance is known (tags in view now, or seen within the hold time). */
    public boolean hasDistance() { return !Double.isNaN(distanceInches); }

    public int getTagCount() { return tagCount; }

    /** Height above the floor (inches) of the tracked CELL's cluster centre in this frame, or NaN. */
    public double getTagHeightInches() { return visible ? tagHeightInches : Double.NaN; }

    /** Ids of the tracked CELL's tags that have a usable 3D position in the current frame. */
    public int[] getTagIds() { return tagIds; }

    /** For each id in {@link #getTagIds()}: how far the tag is in FRONT of the robot centre (metres, floor). */
    public double[] getTagForwardMeters() { return tagForwardMeters; }

    /** For each id in {@link #getTagIds()}: how far the tag is to the RIGHT of the robot centre (metres). */
    public double[] getTagRightMeters() { return tagRightMeters; }

    /** First tag id of the CELL being tracked, or -1. */
    public int getBasketFirstTag() { return basketFirstTag; }
}
