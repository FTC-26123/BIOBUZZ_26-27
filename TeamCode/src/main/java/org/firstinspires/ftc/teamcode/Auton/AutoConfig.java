package org.firstinspires.ftc.teamcode.Auton;

/**
 * Every setting for the one autonomous program, {@link BioBuzzAuto}. Change numbers here; the program itself
 * should not need editing. Setup notes: NOTES/ROBOT-SETUP.md, step 9.
 *
 * <p>Words used here:
 * <ul>
 *   <li>UP side   = we start next to the basket that is already raised, so we can shoot straight away.</li>
 *   <li>DOWN side = we start next to the basket that is down; we wait for the other side to tip it up.</li>
 * </ul>
 */
public final class AutoConfig {
    private AutoConfig() { }

    // =====================================================================
    // WHAT THE MENU SHOWS FIRST (the drive team can change all three during INIT)
    // =====================================================================
    public static final boolean DEFAULT_BLUE = true;
    public static final boolean DEFAULT_UP_SIDE = true;
    /** 0 = shoot + flower + park, 1 = shoot + park, 2 = park only. */
    public static final int DEFAULT_PLAN = 0;

    // =====================================================================
    // ASSUMPTIONS YOU CAN CHANGE
    // Each one is a guess made without the robot. Check them on the field (NOTES/ROBOT-SETUP.md, step 9.2)
    // and change the value here; the program follows whatever is set.
    // =====================================================================

    /**
     * ASSUMPTION 1 - which wall is the UP side.
     * false = the UP side starts at the old "Basket 1" wall mark and the DOWN side at the old "Basket 2" wall
     *         mark (how the earlier programs were set up).
     * true  = the other way round: the UP side starts at the "Basket 2" wall mark and the DOWN side at the
     *         "Basket 1" wall mark. Start mark, shot mark, parking slot and basket tags all follow.
     */
    public static final boolean SWAP_START_WALLS = false;

    /**
     * ASSUMPTION 2 - where the flowers are and which one each start uses.
     * Positions are for the RED alliance in field metres from the field centre (x to the right, y away from
     * the "Basket 1" wall), plus the direction the robot faces when pushed against the flower, in degrees
     * (0 = +x, 90 = +y, 180 = -x). Blue is the same turned half a turn. Taken from the older flower routes.
     */
    public static final double[] FLOWER_1 = {-1.415, -.5942, 180};   // on the side wall, near the Basket 1 wall
    public static final double[] FLOWER_2 = {-.5942, 1.415, 90};     // on the Basket 2 wall
    /** Which flower (1 or 2) a robot that started at the Basket 1 wall mark goes to. */
    public static final int FLOWER_FOR_BASKET1_START = 1;
    /** Which flower (1 or 2) a robot that started at the Basket 2 wall mark goes to. */
    public static final int FLOWER_FOR_BASKET2_START = 2;

    /**
     * ASSUMPTION 3 - which end of the robot goes into the flower.
     * false = front first (the intake end leads). true = back first (the robot reverses into the flower).
     */
    public static final boolean FLOWER_DRIVE_IN_BACKWARDS = false;

    /**
     * ASSUMPTION 4 - when the arm comes down.
     * true  = at the stand-off point, BEFORE driving into the flower.
     * false = drive in with the arm up, then lower it once the robot is against the flower.
     */
    public static final boolean FLOWER_ARM_DOWN_BEFORE_DRIVE_IN = true;

    /**
     * ASSUMPTION 5 - when the intake runs at the flower. It always runs while the robot sits at the flower.
     */
    public static final boolean FLOWER_INTAKE_WHILE_DRIVING_IN = true;
    public static final boolean FLOWER_INTAKE_WHILE_BACKING_OUT = true;
    /** Extra seconds with the intake running after backing out (0 = none), for balls that roll out late. */
    public static final double FLOWER_INTAKE_EXTRA_SEC = 0.0;

    /**
     * ASSUMPTION 6 - the flywheel while the DOWN side waits for the tip.
     * true  = keep it spinning the whole wait, so the first ball leaves the moment the basket is up.
     * false = leave it off and spin up only when the tip is seen (saves battery, costs about a second or two).
     */
    public static final boolean DOWN_SPIN_FLYWHEEL_WHILE_WAITING = true;

    // =====================================================================
    // SHOOTING
    // =====================================================================
    /** Balls loaded by hand before the match. */
    public static final int BALLS = 4;
    /**
     * FILL IN: flywheel speed (ticks/sec) from each shot mark. Used when the camera distance table is not
     * available. 1500 is the old default for both marks.
     */
    public static final double UP_SHOT_SPEED = 1500;
    public static final double DOWN_SHOT_SPEED = 1500;
    /**
     * FILL IN (measure, then lower): the longest the robot may spend shooting, in seconds. 13 is the old
     * allowance. Measure four balls on the robot and set this to the worst time plus about 1.5 s.
     */
    public static final double SHOOT_BUDGET_SEC = 13.0;
    /** While feeding, if no ball has gone through for this long, assume the robot is empty and move on. */
    public static final double NO_SHOT_GIVE_UP_SEC = 2.5;
    /** After the last counted ball, keep the flywheel running this long so the ball is clear. */
    public static final double AFTER_LAST_SHOT_SEC = 0.3;
    /** Feeding is allowed while the robot is within this distance (inches) / angle (degrees) of the shot mark. */
    public static final double ON_MARK_INCHES = 3.0;
    public static final double ON_MARK_DEGREES = 6.0;

    // =====================================================================
    // CAMERA (only used when ShotConfig.AUTO_SHOT_ENABLED is also true)
    // =====================================================================
    /** false = never use the Limelight in autonomous, whatever ShotConfig says. */
    public static final boolean USE_CAMERA = true;
    /** The camera may turn the robot at most this far from the planned shot heading (degrees). */
    public static final double CAMERA_AIM_MAX_DEG = 8.0;
    /**
     * +1 or -1. +1 = when the basket is to the RIGHT in the picture the robot turns clockwise. If camera aiming
     * turns the robot away from the basket, set -1.
     */
    public static final double CAMERA_TURN_SIGN = 1.0;
    /** On arrival, wait up to this long for the camera to see the basket before shooting on odometry alone. */
    public static final double CAMERA_WAIT_SEC = 1.0;
    /** If the camera has not lined the robot up after this long, go back to the planned heading and shoot. */
    public static final double CAMERA_AIM_GIVE_UP_SEC = 2.5;

    // =====================================================================
    // CAMERA POSITION FIX at the shot mark (needs the camera on)
    // While the robot sits at the shot mark looking at the raised basket, the camera works out where the robot
    // really is from the basket's tags and compares it with the odometry. The difference is always shown on the
    // Driver Station ("Camera position check"). With the switch below on, the odometry is corrected by that
    // difference, so the drive to the flower and to the parking slot starts from the true position.
    // Only x/y are corrected; the heading stays the odometry's.
    // =====================================================================
    /**
     * false = measure and show the difference only (use this first, to check the numbers below).
     * true  = also correct the odometry. Turn on only after NOTES/ROBOT-SETUP.md step 9.6 passes.
     */
    public static final boolean CAMERA_FIX_POSITION = false;
    /** A correction larger than this (inches) is not believed and not applied. */
    public static final double CAMERA_FIX_MAX_IN = 6.0;
    /** A difference smaller than this (inches) is not worth applying. */
    public static final double CAMERA_FIX_MIN_IN = 0.3;
    /** At least this many of the basket's four tags must be measured in a frame. */
    public static final int CAMERA_FIX_MIN_TAGS = 2;
    /** Number of still camera frames averaged (the middle value is used). */
    public static final int CAMERA_FIX_SAMPLES = 8;
    /** The frames must agree with each other within this (inches) or nothing is applied. */
    public static final double CAMERA_FIX_SPREAD_IN = 2.0;
    /**
     * FILL IN (check on the real field): where the basket tags are when the basket is UP, for RED, in metres
     * from the field centre. Taken from the older autonomous code's field model.
     *   BASKET_TAG_CENTER_X_M  : x of the middle of the row of four tags (both baskets).
     *   BASKET_TAG_Y_UP_M      : distance of the tag row from the field's centre line (y). The Basket 1 end is
     *                            at minus this, the Basket 2 end at plus this.
     *   BASKET_TAG_OFFSETS_IN  : each tag's position along the row from its middle, in inches, in id order.
     */
    public static final double BASKET_TAG_CENTER_X_M = -.32385;
    public static final double BASKET_TAG_Y_UP_M = .327341117;
    public static final double[] BASKET_TAG_OFFSETS_IN = {-6.5, -2.75, 2.75, 6.5};

    // =====================================================================
    // DOWN SIDE: waiting for the basket to come up
    // =====================================================================
    /**
     * Latest time on the autonomous clock (seconds) to keep waiting for the tip. The robot also stops waiting
     * by itself once there would not be {@link #MIN_SHOOT_SEC} left to shoot and still reach the parking slot
     * (about 22-23 s), so 30 means "wait as long as it is still useful". Lower it to leave earlier.
     */
    public static final double DOWN_MAX_WAIT_SEC = 30.0;
    /**
     * FILL IN (measure): our basket counts as UP when its AprilTags are higher than this above the floor
     * (inches). The autonomous code's field model has the tags near 34-39 in when down and 48-53 in when up, so
     * the default is the midpoint. Check both numbers on the real field (telemetry "Tag height").
     */
    public static final double BASKET_UP_TAG_HEIGHT_IN = 43.7;
    /** The basket must look UP for this long without a break before we shoot (it swings when it tips). */
    public static final double TIP_CONFIRM_SEC = 0.65;
    /**
     * With NO camera we cannot see the tip. Shoot anyway at this time on the autonomous clock (the old
     * "wait 15" behaviour, which relies on the teammate), or -1 to never shoot without a camera.
     */
    public static final double DOWN_NO_CAMERA_SHOOT_AT_SEC = 15.0;
    /** Do not start shooting if less than this many seconds would be left for it. */
    public static final double MIN_SHOOT_SEC = 4.0;

    // =====================================================================
    // FLOWER (needs FlowerArm.ENABLED = true, otherwise this part is skipped)
    // =====================================================================
    /**
     * FILL IN (measure): how far back from the flower (inches) the robot stops to lower the arm before driving
     * in. Must be far enough that the arm can swing down without hitting the flower bed.
     */
    public static final double FLOWER_STANDOFF_IN = 12.0;
    /** Time for the arm to finish swinging down before the robot moves in (seconds). */
    public static final double FLOWER_ARM_DOWN_SEC = 0.5;
    /** Speed of the push into the flower bed and of the retreat (metres per second). */
    public static final double FLOWER_PUSH_M_PER_S = 0.25;
    /** FILL IN (measure): how long to sit at the flower with the intake running while the balls come out. */
    public static final double FLOWER_COLLECT_SEC = 2.5;

    // =====================================================================
    // TIME LIMITS
    // =====================================================================
    /** The robot must be parked by this time on the autonomous clock. */
    public static final double PARK_BY_SEC = 29.5;
    /** Everything is switched off at this time no matter what. */
    public static final double HARD_STOP_SEC = 29.7;
    /** Extra safety margin on top of the calculated driving time to the parking slot. */
    public static final double PARK_MARGIN_SEC = 0.5;
    /** A drive leg that takes longer than this accepts a wider arrival window instead of waiting forever. */
    public static final double LEG_LATE_SEC = 4.0;
}
