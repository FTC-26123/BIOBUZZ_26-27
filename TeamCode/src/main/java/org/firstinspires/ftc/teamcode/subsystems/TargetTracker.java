package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Auton.cycles.LimelightMount;

import java.util.Arrays;
import java.util.List;

/**
 * Finds our alliance's basket in a Limelight frame and reports two numbers:
 * <ul>
 *   <li>aim error in degrees (0 = the robot points at the basket, positive = basket is to the right)</li>
 *   <li>floor distance in inches from the robot's centre to the basket's tags</li>
 * </ul>
 * Each basket carries four AprilTags in a row. If both of our baskets are in view, the one closer to the
 * middle of the picture (the one the driver is pointing at) is used.
 *
 * <p>The distance comes from each tag's measured 3D position, tilted back level using the camera pitch in
 * {@link LimelightMount}. It does not depend on how high the basket is, so a raised or lowered basket gives
 * the same floor distance.
 */
public final class TargetTracker {
    private static final int HISTORY = 5;
    private final double[] history = new double[HISTORY];
    private int historyCount = 0, historyNext = 0;

    private boolean visible = false;
    private double aimErrorDeg = 0;
    private double distanceInches = Double.NaN;
    private double lastSeenSec = Double.NEGATIVE_INFINITY;
    private int tagCount = 0, basketFirstTag = -1;

    /**
     * Call once per loop.
     *
     * @param frame  latest Limelight result (may be null)
     * @param blue   true = blue alliance tags, false = red
     * @param nowSec any running clock in seconds
     */
    public void update(LLResult frame, boolean blue, double nowSec) {
        visible = false;
        tagCount = 0;
        List<LLResultTypes.FiducialResult> tags = null;
        if (frame != null && frame.isValid() && frame.getStaleness() <= ShotConfig.MAX_FRAME_AGE_MS) {
            tags = frame.getFiducialResults();
        }
        if (tags != null && !tags.isEmpty()) {
            int first = blue ? ShotConfig.BLUE_FIRST_TAG : ShotConfig.RED_FIRST_TAG;
            int baskets = ShotConfig.BASKETS_PER_ALLIANCE, per = ShotConfig.TAGS_PER_BASKET;
            double[] sumTx = new double[baskets], sumDist = new double[baskets];
            int[] count = new int[baskets], distCount = new int[baskets];
            for (LLResultTypes.FiducialResult tag : tags) {
                int index = tag.getFiducialId() - first;
                if (index < 0 || index >= baskets * per) continue;   // not one of our basket tags
                int basket = index / per;
                sumTx[basket] += tag.getTargetXDegrees();
                count[basket]++;
                double d = floorDistanceInches(tag.getTargetPoseCameraSpace());
                if (!Double.isNaN(d)) { sumDist[basket] += d; distCount[basket]++; }
            }
            int best = -1;
            for (int b = 0; b < baskets; b++) {
                if (count[b] == 0) continue;
                if (best < 0 || Math.abs(sumTx[b] / count[b]) < Math.abs(sumTx[best] / count[best])) best = b;
            }
            if (best >= 0) {
                visible = true;
                tagCount = count[best];
                if (basketFirstTag != first + best * per) historyCount = historyNext = 0;   // switched basket
                basketFirstTag = first + best * per;
                aimErrorDeg = sumTx[best] / count[best] + ShotConfig.AIM_OFFSET_DEG;
                if (distCount[best] > 0) {
                    history[historyNext] = sumDist[best] / distCount[best];
                    historyNext = (historyNext + 1) % HISTORY;
                    historyCount = Math.min(HISTORY, historyCount + 1);
                    distanceInches = median();
                }
                lastSeenSec = nowSec;
            }
        }
        if (!visible && nowSec - lastSeenSec > ShotConfig.DISTANCE_HOLD_SEC) {
            historyCount = historyNext = 0;
            distanceInches = Double.NaN;
        }
    }

    /**
     * Floor distance from the robot's centre to one tag. Limelight camera axes: x = right, y = down,
     * z = forward (along the lens). The camera looks upward by the mount pitch, so the level forward distance
     * is z*cos(pitch) + y*sin(pitch).
     */
    private static double floorDistanceInches(Pose3D tagInCamera) {
        if (tagInCamera == null) return Double.NaN;
        Position p = tagInCamera.getPosition();
        if (p == null) return Double.NaN;
        p = p.toUnit(DistanceUnit.METER);
        if (Double.isNaN(p.x + p.y + p.z) || p.z <= 0.05) return Double.NaN;
        double pitch = Math.toRadians(LimelightMount.PITCH_DEGREES);
        double forward = p.z * Math.cos(pitch) + p.y * Math.sin(pitch);
        if (forward <= 0.1) return Double.NaN;
        double meters = Math.hypot(forward + LimelightMount.FORWARD_METERS, p.x + LimelightMount.RIGHT_METERS);
        return meters > 5 ? Double.NaN : meters / 0.0254;
    }

    private double median() {
        double[] sorted = Arrays.copyOf(history, historyCount);
        Arrays.sort(sorted);
        return sorted[historyCount / 2];
    }

    /** @return true if our basket's tags are in the current camera frame. */
    public boolean isVisible() { return visible; }

    /** Degrees the robot must turn; positive = basket is to the right. Only meaningful while visible. */
    public double getAimErrorDeg() { return aimErrorDeg; }

    /** Floor distance in inches (median of the last few frames), or NaN if not known. */
    public double getDistanceInches() { return distanceInches; }

    /** @return true if a distance is known (tags in view now, or seen within the hold time). */
    public boolean hasDistance() { return !Double.isNaN(distanceInches); }

    public int getTagCount() { return tagCount; }

    /** First tag id of the basket being tracked, or -1. */
    public int getBasketFirstTag() { return basketFirstTag; }
}
