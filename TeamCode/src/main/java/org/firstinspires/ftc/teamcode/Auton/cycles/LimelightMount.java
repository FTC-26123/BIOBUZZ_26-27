package org.firstinspires.ftc.teamcode.Auton.cycles;

/** Lens position relative to the Pinpoint pose origin, in meters. */
public final class LimelightMount {
    /** MEASURED on the robot: floor to lens centre (12 3/8 in) and lens left/right of centre (centred). */
    public static final double MEASURED_HEIGHT_IN = 12.375, MEASURED_RIGHT_IN = 0;
    public static double HEIGHT_METERS = MEASURED_HEIGHT_IN * .0254;
    /** Camera tilt above level. Set by the "Limelight Calibration" OpMode (two taped distances), not by hand. */
    public static double PITCH_DEGREES = 43;
    /** Robot centre to lens, forward. Set by the "Limelight Calibration" OpMode, not by hand. */
    public static double FORWARD_METERS = 7 * .0254;
    public static double RIGHT_METERS = MEASURED_RIGHT_IN * .0254;
    private LimelightMount() { }
    public static boolean calibrated(){return Double.isFinite(FORWARD_METERS);}
}
