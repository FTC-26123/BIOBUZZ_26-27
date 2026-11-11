package org.firstinspires.ftc.teamcode.subsystems;

/**
 * ALL the numbers and switches for camera aiming and distance-based shot speed in TeleOp, in one place.
 *
 * <p>Everything marked FILL IN has to be measured on the robot. Step-by-step instructions are in
 * NOTES/ROBOT-SETUP.md (step 8). Until {@link #AUTO_SHOT_ENABLED} is set to true none of this runs and TeleOp
 * behaves exactly as it does with manual presets.
 *
 * <p>The camera mount numbers (height, forward offset, pitch) are NOT here: they live in
 * {@code Auton/cycles/LimelightMount.java} because autonomous uses the same ones.
 */
public final class ShotConfig {
    private ShotConfig() { }

    // =====================================================================
    // SWITCHES
    // =====================================================================

    /**
     * MASTER SWITCH. false = the Limelight is not even started and the aim button does nothing.
     * Set true only after Phases 1 and 2 have been checked on the robot (NOTES/ROBOT-SETUP.md, steps 1-5).
     */
    public static final boolean AUTO_SHOT_ENABLED = false;

    /**
     * Shot speed from distance. Needs {@link #SHOT_TABLE} to have at least two rows; with an empty table the
     * manual presets are used no matter what this says.
     */
    public static final boolean AUTO_SPEED_ENABLED = true;

    /**
     * While the AIM button is held, only feed balls when the robot is pointed at the target
     * (within {@link #AIM_FIRE_TOLERANCE_DEG}). Firing WITHOUT the aim button held is never blocked.
     */
    public static final boolean AIM_GATE_ENABLED = true;

    // =====================================================================
    // CAMERA
    // =====================================================================

    /**
     * FILL IN: the Limelight pipeline number that detects AprilTags (check the Limelight web page).
     * TeleOp used 0 before; the autonomous code uses 1 (VisionRules.SHOT_PIPELINE). They must be the same pipeline.
     */
    public static final int APRILTAG_PIPELINE = 0;

    /** Camera frames older than this are ignored (milliseconds). */
    public static final double MAX_FRAME_AGE_MS = 150;

    /** After the tags leave the picture, the last distance is still trusted for this long (seconds). */
    public static final double DISTANCE_HOLD_SEC = 1.0;

    // =====================================================================
    // TARGET TAGS  (ids taken from the autonomous code: each basket carries 4 tags in a row)
    // =====================================================================

    /** Alliance used if nobody presses X (blue) or B (red) on gamepad 1 during INIT. */
    public static final boolean DEFAULT_ALLIANCE_BLUE = true;
    /** Blue baskets: tags 38-41 and 42-45. */
    public static final int BLUE_FIRST_TAG = 38;
    /** Red baskets: tags 30-33 and 34-37. */
    public static final int RED_FIRST_TAG = 30;
    public static final int TAGS_PER_BASKET = 4;
    public static final int BASKETS_PER_ALLIANCE = 2;

    // =====================================================================
    // AIMING (turning the robot toward the basket)
    // =====================================================================

    /**
     * FILL IN (tune): turn power per degree of aim error. Too low = slow to line up; too high = the robot
     * swings back and forth. NOTE: the drive speed setting (25/50/75/100%) also scales this, so tune it at the
     * drive speed you use when shooting.
     */
    public static final double AIM_KP = 0.02;
    /** FILL IN (tune): damping. Raise in small steps (0.001) if the robot overshoots and swings back. */
    public static final double AIM_KD = 0.0;
    /**
     * FILL IN (check): +1 or -1. If holding AIM makes the robot turn AWAY from the basket, change this to -1.
     */
    public static final double AIM_DIRECTION = 1.0;
    /** The robot stops turning once the aim error is below this (degrees). */
    public static final double AIM_STOP_DEG = 0.5;
    /** FILL IN (tune): balls are only fed while the aim error is below this (degrees), when AIM is held. */
    public static final double AIM_FIRE_TOLERANCE_DEG = 2.0;
    /**
     * FILL IN (tune): if shots land consistently left or right when the camera says "aimed", put the correction
     * here in degrees. Positive aims further to the RIGHT.
     */
    public static final double AIM_OFFSET_DEG = 0.0;

    // =====================================================================
    // DISTANCE -> FLYWHEEL SPEED
    // =====================================================================

    /**
     * FILL IN: measured shot table. Each row is { distance in inches, flywheel speed in ticks/sec }.
     * The distance is the "Distance (in)" number shown on the Driver Station while the tags are in view.
     * Rows may be in any order; speeds between two rows are interpolated. Example of the format:
     *
     * <pre>
     *     public static final double[][] SHOT_TABLE = {
     *             { 30, 1450 },
     *             { 42, 1520 },
     *             { 54, 1610 },
     *             { 66, 1720 },
     *             { 78, 1850 },
     *     };
     * </pre>
     * Those numbers are only an illustration of the layout - do NOT use them. Leave the table empty until you
     * have measured real values; an empty table simply keeps the manual presets.
     */
    public static final double[][] SHOT_TABLE = {
    };

    /** The auto speed only changes in steps of this size (ticks/sec) so camera noise does not jiggle the target. */
    public static final double AUTO_SPEED_STEP = 10;
}
