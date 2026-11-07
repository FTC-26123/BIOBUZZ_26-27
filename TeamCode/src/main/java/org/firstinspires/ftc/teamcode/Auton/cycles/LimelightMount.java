package org.firstinspires.ftc.teamcode.Auton.cycles;

/** Lens position relative to the Pinpoint pose origin, in meters. */
public final class LimelightMount {
    public static final double HEIGHT_METERS = 10 * .0254;
    public static final double PITCH_DEGREES = 43;
    // Measure origin-to-lens distance along the robot's forward axis before shooting.
    public static double FORWARD_METERS = 7 * .0254;
    public static final double RIGHT_METERS = 0; // Centered across the robot's front.
    private LimelightMount() { }
    public static boolean calibrated(){return Double.isFinite(FORWARD_METERS);}
}
