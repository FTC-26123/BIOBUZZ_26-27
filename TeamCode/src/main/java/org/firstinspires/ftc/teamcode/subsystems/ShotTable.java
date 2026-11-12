package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Arrays;

/** Distance (inches) to flywheel speed (ticks/sec), straight-line interpolation between measured rows. */
public final class ShotTable {
    private final double[][] rows;

    public ShotTable(double[][] measured) {
        double[][] copy = measured == null ? new double[0][] : measured.clone();
        int valid = 0;
        for (double[] r : copy) {
            if (r != null && r.length >= 2 && r[0] > 0 && r[1] > 0
                    && !Double.isNaN(r[0]) && !Double.isNaN(r[1])) copy[valid++] = r;
        }
        rows = Arrays.copyOf(copy, valid);
        Arrays.sort(rows, (a, b) -> Double.compare(a[0], b[0]));
    }

    /** At least two rows are needed before the table is used. */
    public boolean isReady() { return rows.length >= 2; }

    public int size() { return rows.length; }
    public double minDistance() { return isReady() ? rows[0][0] : Double.NaN; }
    public double maxDistance() { return isReady() ? rows[rows.length - 1][0] : Double.NaN; }

    /** @return true if the distance is between the nearest and farthest measured rows. */
    public boolean inRange(double inches) {
        return isReady() && inches >= rows[0][0] && inches <= rows[rows.length - 1][0];
    }

    /** Speed for a distance. Outside the table the nearest end row is used (no guessing beyond it). */
    public double speedFor(double inches) {
        if (!isReady() || Double.isNaN(inches)) return Double.NaN;
        if (inches <= rows[0][0]) return rows[0][1];
        for (int i = 1; i < rows.length; i++) {
            if (inches <= rows[i][0]) {
                double span = rows[i][0] - rows[i - 1][0];
                double f = span <= 0 ? 1 : (inches - rows[i - 1][0]) / span;
                return rows[i - 1][1] + f * (rows[i][1] - rows[i - 1][1]);
            }
        }
        return rows[rows.length - 1][1];
    }
}
