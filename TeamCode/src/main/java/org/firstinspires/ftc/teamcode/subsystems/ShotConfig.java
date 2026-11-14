package org.firstinspires.ftc.teamcode.subsystems;

/**
 * ALL the numbers and switches for camera aiming and distance-based shot speed in TeleOp, in one place.
 *
 * <p>These are only the CODE DEFAULTS. The values used on the robot are edited with the "Robot Settings" OpMode
 * (groups Camera and Aim) and the table with the "Shot Table Builder" OpMode; both save on the Control Hub, so
 * nothing here needs to be changed or re-uploaded.
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
     * MASTER SWITCH. Turn it on with Robot Settings > Camera > Camera enabled (no code change).
     * (In the desktop simulator it is always on, so it can be tried there first: "|| InSimulator.YES".)
     * false = the Limelight is not even started and the aim button does nothing.
     * Set true only after Phases 1 and 2 have been checked on the robot (NOTES/ROBOT-SETUP.md, steps 1-5).
     */
    public static boolean AUTO_SHOT_ENABLED = false || InSimulator.YES;

    /**
     * Shot speed from distance. Needs {@link #SHOT_TABLE} to have at least two rows; with an empty table the
     * manual presets are used no matter what this says.
     */
    public static boolean AUTO_SPEED_ENABLED = true;

    /**
     * While the AIM button is held, only feed balls when the robot is pointed at the target
     * (within {@link #AIM_FIRE_TOLERANCE_DEG}). Firing WITHOUT the aim button held is never blocked.
     */
    public static boolean AIM_GATE_ENABLED = true;

    // =====================================================================
    // CAMERA
    // =====================================================================

    /**
     * FILL IN: the Limelight pipeline number that detects AprilTags (check the Limelight web page).
     * 1 is what the older autonomous code uses (VisionRules.SHOT_PIPELINE) and what the simulator's AprilTag
     * pipeline is; the old TeleOp code used 0. Set it to whatever slot holds the AprilTag pipeline on your camera.
     */
    public static int APRILTAG_PIPELINE = 1;

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
    public static int BLUE_FIRST_TAG = 38;
    /** Red baskets: tags 30-33 and 34-37. */
    public static int RED_FIRST_TAG = 30;
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
    public static double AIM_KP = 0.02;
    /** FILL IN (tune): damping. Raise in small steps (0.001) if the robot overshoots and swings back. */
    public static double AIM_KD = 0.0;
    /**
     * FILL IN (check): +1 or -1. If holding AIM makes the robot turn AWAY from the basket, change this to -1.
     */
    public static double AIM_DIRECTION = 1.0;
    /**
     * Most turn power AIM may use (0..1). "How To Auto-Rotate to an AprilTag" (B. Pratt) clamps to 0.4 so a big
     * error can never whip the robot past the target.
     */
    public static double AIM_MAX_POWER = 0.4;
    /** Smallest turn power while outside the stop angle, so the robot does not stall just short. 0 = off. */
    public static double AIM_MIN_POWER = 0.0;
    /** The robot stops turning once the aim error is below this (degrees). */
    public static double AIM_STOP_DEG = 0.5;
    /** FILL IN (tune): balls are only fed while the aim error is below this (degrees), when AIM is held. */
    public static double AIM_FIRE_TOLERANCE_DEG = 2.0;

    // =====================================================================
    // SHOOTING RANGE (used by the LEDs: "can we score from where we are standing?")
    // =====================================================================
    /** FILL IN (measure): closer than this (inches, camera distance) the ball cannot drop in - e.g. right under the basket. */
    public static double MIN_SHOT_DISTANCE_IN = 24.0;
    /** FILL IN (measure): farther than this (inches) shots stop being reliable. */
    public static double MAX_SHOT_DISTANCE_IN = 120.0;

    // =====================================================================
    // AIM POINT inside the upward CELL (see TargetTracker). Always dead centre left-right.
    // =====================================================================
    /** TUNE: where in the opening to aim, 0 = bottom lip, 1 = top. Depends on our shooter angle. */
    public static double AIM_HEIGHT_FRACTION = 0.4;
    /** How far inside the CELL past the opening to aim (inches). Normally 0. */
    public static double AIM_DEPTH_IN = 0.0;
    /** Game manual: cluster centre to the opening, along the CELL's bottom face (9.94 - 2.75 in). */
    public static double CLUSTER_TO_OPENING_IN = 7.19;
    /** Game manual Fig. 9-11: opening height, bottom lip to top. */
    public static double OPENING_HEIGHT_IN = 14.0;
    /** Game manual Fig. 9-10: HIVE resting tilt. */
    public static double HIVE_TILT_DEG = 30.0;

    // =====================================================================
    // SHOT MODEL (see ShotModel). Measure the angle and exit height on the robot; scale is tuned on the field.
    // =====================================================================
    public static boolean USE_SHOT_MODEL = true;
    /** MEASURE: angle the ball leaves the shooter, degrees above level. */
    public static double LAUNCH_ANGLE_DEG = 55.0;
    /** MEASURE: height of the ball's exit point above the floor (inches). */
    public static double EXIT_HEIGHT_IN = 12.0;
    /** MEASURE: how far the exit point is in front of the robot centre (inches). */
    public static double EXIT_FORWARD_IN = 0.0;
    /** TUNE (G2 right stick button in Shot Table Builder): flywheel ticks/sec per inch/sec of ball speed. */
    public static double TICKS_PER_IN_PER_SEC = 7.0;

    // =====================================================================
    // DISTANCE -> FLYWHEEL SPEED
    // =====================================================================

    /**
     * Code default for the shot table. Normally left empty: record the real rows on the field with the
     * "Shot Table Builder" OpMode, which saves them on the robot.
     * Measured shot table. Each row is { distance in inches, flywheel speed in ticks/sec }.
     * The distance is the "Distance (in)" number shown on the Driver Station while the tags are in view.
     * Rows may be in any order; speeds between two rows are interpolated. Example of the format:
     *
     * <pre>
     *     public static double[][] SHOT_TABLE = {
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
    public static double[][] SHOT_TABLE = {
    };

    /** The auto speed only changes in steps of this size (ticks/sec) so camera noise does not jiggle the target. */
    public static double AUTO_SPEED_STEP = 10;
}
