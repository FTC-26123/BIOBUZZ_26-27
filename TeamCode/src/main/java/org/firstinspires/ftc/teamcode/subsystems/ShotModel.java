package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Flywheel speed for a shot at the aim point, from simple projectile physics plus the measured shot table.
 *
 * <p>Model: the ball leaves the shooter at {@link ShotConfig#LAUNCH_ANGLE_DEG} from level, from
 * {@link ShotConfig#EXIT_HEIGHT_IN} above the floor and {@link ShotConfig#EXIT_FORWARD_IN} in front of the robot
 * centre. The ball speed needed to pass through a point d inches away and h inches higher is
 * v = sqrt(g d^2 / (2 cos^2(angle) (d tan(angle) - h))). Flywheel ticks/sec = v x
 * {@link ShotConfig#TICKS_PER_IN_PER_SEC}.
 *
 * <p>Table: every recorded row (distance, ticks/sec that scored) CORRECTS the model. At a recorded distance the
 * result is exactly the recorded speed; between rows the correction is interpolated, so the model supplies the
 * shape and the rows pin it to reality. With the model off, the table alone is used (needs 2 rows).
 */
public final class ShotModel {
    private ShotModel() { }

    private static final double G_IN_PER_S2 = 386.09;

    /** Ball speed (in/s) the model says is needed, or NaN if the shot is impossible at this angle. */
    public static double ballSpeed(double distanceIn, double aimHeightIn) {
        double d = distanceIn - ShotConfig.EXIT_FORWARD_IN;
        double h = aimHeightIn - ShotConfig.EXIT_HEIGHT_IN;
        double a = Math.toRadians(ShotConfig.LAUNCH_ANGLE_DEG);
        double cos = Math.cos(a);
        double denom = 2 * cos * cos * (d * Math.tan(a) - h);
        if (Double.isNaN(d + h) || d <= 0 || denom <= 0) return Double.NaN;
        return Math.sqrt(G_IN_PER_S2 * d * d / denom);
    }

    /** Model-only flywheel speed (ticks/sec), or NaN. */
    public static double modelSpeed(double distanceIn, double aimHeightIn) {
        return ballSpeed(distanceIn, aimHeightIn) * ShotConfig.TICKS_PER_IN_PER_SEC;
    }

    /**
     * Final flywheel speed (ticks/sec) for this distance and aim height, or NaN if nothing can say.
     * Model + table correction when the model is on; the table alone (2+ rows) when it is off.
     */
    public static double speedFor(double distanceIn, double aimHeightIn, ShotTable table) {
        if (Double.isNaN(distanceIn)) return Double.NaN;
        double model = ShotConfig.USE_SHOT_MODEL ? modelSpeed(distanceIn, aimHeightIn) : Double.NaN;
        if (Double.isNaN(model)) return table.isReady() ? table.speedFor(distanceIn) : Double.NaN;
        double[][] rows = ShotConfig.SHOT_TABLE;
        if (rows.length == 0) return model;
        // Correction = recorded - model at each row; interpolate it at this distance (ends held flat).
        double[] dist = new double[rows.length], corr = new double[rows.length];
        int n = 0;
        for (double[] r : rows) {
            double m = modelSpeed(r[0], aimHeightIn);
            if (r.length < 2 || Double.isNaN(m)) continue;
            dist[n] = r[0];
            corr[n] = r[1] - m;
            n++;
        }
        if (n == 0) return model;
        double c;
        if (distanceIn <= dist[0]) c = corr[0];
        else if (distanceIn >= dist[n - 1]) c = corr[n - 1];
        else {
            c = corr[n - 1];
            for (int i = 1; i < n; i++) {
                if (distanceIn <= dist[i]) {
                    double f = (distanceIn - dist[i - 1]) / Math.max(1e-6, dist[i] - dist[i - 1]);
                    c = corr[i - 1] + f * (corr[i] - corr[i - 1]);
                    break;
                }
            }
        }
        return model + c;
    }

    /** true if a speed can be produced (model on, or table has 2+ rows). */
    public static boolean available(ShotTable table) {
        return ShotConfig.USE_SHOT_MODEL || table.isReady();
    }

    /**
     * Sets {@link ShotConfig#TICKS_PER_IN_PER_SEC} so the model gives exactly this speed here ("scale the model to
     * the speed that just scored"). @return false if the model has no answer at this distance/height.
     */
    public static boolean scaleTo(double ticksPerSec, double distanceIn, double aimHeightIn) {
        double v = ballSpeed(distanceIn, aimHeightIn);
        if (Double.isNaN(v) || v <= 0 || ticksPerSec <= 0) return false;
        ShotConfig.TICKS_PER_IN_PER_SEC = ticksPerSec / v;
        return true;
    }
}
