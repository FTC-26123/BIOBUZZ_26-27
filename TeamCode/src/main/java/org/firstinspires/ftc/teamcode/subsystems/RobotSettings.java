package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Auton.AutoConfig;
import org.firstinspires.ftc.teamcode.Auton.cycles.LimelightMount;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Properties;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Every number the team tunes on the robot, saved in a file on the Control Hub so it can be changed from the
 * Driver Station (the "Robot Settings" OpMode, or the tuning OpModes) WITHOUT editing code or re-uploading.
 *
 * <p>How it works: the values written in the code (Shooter.kP, AutoConfig.*, ShotConfig.*, ...) are only the
 * defaults. When the Robot Controller app starts, the saved file is read and those static fields are overwritten.
 * Editing a setting changes the field immediately and saves the file, so the next OpMode uses it.
 *
 * <p>To add a new setting: make the field {@code public static} (not final) and add one line in {@link #build()}.
 */
public final class RobotSettings {
    private RobotSettings() { }

    public enum Kind { NUMBER, INT, BOOL }

    /** One editable value. */
    public static final class Setting {
        public final String group, label, help;
        public final Kind kind;
        public final double step, min, max;
        private final DoubleSupplier getter;
        private final DoubleConsumer setter;
        private double codeDefault;

        Setting(String group, String label, Kind kind, double step, double min, double max, String help,
                DoubleSupplier getter, DoubleConsumer setter) {
            this.group = group;
            this.label = label;
            this.kind = kind;
            this.step = step;
            this.min = min;
            this.max = max;
            this.help = help;
            this.getter = getter;
            this.setter = setter;
        }

        public String key() { return (group + "." + label).replace(' ', '_'); }

        public double get() { return getter.getAsDouble(); }

        /** Sets the value, clamped to its range (whole numbers for INT, 0/1 for BOOL). Does not save. */
        public void set(double v) {
            if (Double.isNaN(v)) return;
            v = Math.max(min, Math.min(max, v));
            if (kind == Kind.INT) v = Math.round(v);
            if (kind == Kind.BOOL) v = v >= 0.5 ? 1 : 0;
            setter.accept(v);
        }

        public double codeDefault() { return codeDefault; }

        public boolean isDefault() { return Math.abs(get() - codeDefault) < 1e-12; }

        public String text() { return format(get()); }

        public String format(double v) {
            switch (kind) {
                case BOOL: return v >= 0.5 ? "ON" : "OFF";
                case INT:  return String.format(Locale.US, "%d", Math.round(v));
                default:   return trim(v);
            }
        }
    }

    private static final List<Setting> ALL = new ArrayList<>();
    private static final List<String> GROUPS = new ArrayList<>();
    private static double[][] codeDefaultTable;
    private static boolean loaded = false;
    private static String lastProblem = "";

    static {
        build();
        for (Setting s : ALL) s.codeDefault = s.get();
        codeDefaultTable = copyTable(ShotConfig.SHOT_TABLE);
    }

    // =====================================================================
    // The list of settings. group, label, kind, step, min, max, help, getter, setter
    // =====================================================================
    private static void build() {
        // ---------------- Shooter ----------------
        String g = "Shooter PIDF";
        num(g, "kS", 0.01, 0, 0.5, "Static: power that JUST starts the wheel turning from stopped",
                () -> Shooter.kS, v -> Shooter.kS = v);
        num(g, "kV", 0.00001, 0, 0.01, "Velocity feedforward: power per tick/sec. kS+kV alone gets just under target",
                () -> Shooter.kF, v -> Shooter.kF = v);
        num(g, "kP", 0.0001, 0, 1, "Proportional: fast recovery. Too high = overshoot or humming",
                () -> Shooter.kP, v -> Shooter.kP = v);
        num(g, "kI", 0.0001, 0, 1, "Leave 0 on a flywheel (causes big swings)",
                () -> Shooter.kI, v -> Shooter.kI = v);
        num(g, "kD", 0.0001, 0, 1, "Leave 0 on a flywheel (too much damping)",
                () -> Shooter.kD, v -> Shooter.kD = v);

        g = "Shooter speeds";
        for (int i = 0; i < Shooter.LAUNCHING_POWERS.length; i++) {
            final int k = i;
            num(g, "Preset " + (i + 1), 25, 0, 2400, "Flywheel speed of preset " + (i + 1) + " (ticks/sec)",
                    () -> Shooter.LAUNCHING_POWERS[k], v -> Shooter.LAUNCHING_POWERS[k] = v);
        }
        num(g, "Trim step", 5, 0, 200, "How much one trim button press changes the speed (ticks/sec)",
                () -> Shooter.TRIM_STEP, v -> Shooter.TRIM_STEP = v);

        g = "Shooter gate";
        num(g, "Shot drop", 5, 20, 300, "Speed drop that counts as a ball going through. Lower if balls are missed",
                () -> Shooter.SHOT_DROP, v -> Shooter.SHOT_DROP = v);
        num(g, "Wait between balls s", 0.02, 0, 1, "Extra wait after the speed is back before the next ball",
                () -> Shooter.BETWEEN_BALLS_SEC, v -> Shooter.BETWEEN_BALLS_SEC = v);
        num(g, "Resume below", 5, 0, 300, "Feed reopens once speed >= target - this. Lower if later balls fall short",
                () -> Shooter.RESUME_BELOW_TARGET, v -> Shooter.RESUME_BELOW_TARGET = v);
        num(g, "Resume above", 5, 0, 300, "Feed reopens only while speed <= target + this",
                () -> Shooter.RESUME_ABOVE_TARGET, v -> Shooter.RESUME_ABOVE_TARGET = v);
        num(g, "Pause below", 5, 0, 500, "Feed always pauses when speed < target - this",
                () -> Shooter.PAUSE_BELOW_TARGET, v -> Shooter.PAUSE_BELOW_TARGET = v);
        num(g, "Pause above", 5, 0, 500, "Feed pauses when speed > target + this",
                () -> Shooter.PAUSE_ABOVE_TARGET, v -> Shooter.PAUSE_ABOVE_TARGET = v);
        num(g, "Clear reverse s", 0.01, 0, 1, "Reverse windmill+transfer this long before a volley",
                () -> Shooter.CLEAR_REVERSE_SEC, v -> Shooter.CLEAR_REVERSE_SEC = v);
        num(g, "Windmill spinup s", 0.01, 0, 1, "Windmill head start before the first ball",
                () -> Shooter.WINDMILL_SPINUP_SEC, v -> Shooter.WINDMILL_SPINUP_SEC = v);
        num(g, "Windmill resume s", 0.01, 0, 1, "Windmill head start between balls (raise if it stalls on balls 2-4)",
                () -> Shooter.WINDMILL_RESUME_SEC, v -> Shooter.WINDMILL_RESUME_SEC = v);

        // ---------------- Intake ----------------
        g = "Intake";
        num(g, "Transfer intake pwr", 0.05, 0, 1, "Transfer power while intaking. 0 = off",
                () -> Intake.TRANSFER_INTAKE_POWER, v -> Intake.TRANSFER_INTAKE_POWER = v);
        // Windmill = 312 RPM motor. It always FEEDS at full power; these are its slow reverse speeds (0-312 RPM).
        num(g, "Windmill intake rev RPM", 5, 0, 312, "Windmill reverse while intaking (312 RPM motor)",
                () -> Intake.WINDMILL_INTAKE_REVERSE_RPM, v -> Intake.WINDMILL_INTAKE_REVERSE_RPM = v);
        num(g, "Windmill hold rev RPM", 5, 0, 312, "Windmill reverse between shots. 0 = brake. Reversing can push balls out",
                () -> Intake.WINDMILL_HOLD_REVERSE_RPM, v -> Intake.WINDMILL_HOLD_REVERSE_RPM = v);
        num(g, "Transfer hold pwr", 0.05, 0, 1, "Transfer power between shots (0 = stopped)",
                () -> Intake.TRANSFER_HOLD_POWER, v -> Intake.TRANSFER_HOLD_POWER = v);
        num(g, "Transfer amp limit", 0.5, 0, 20, "Transfer stops briefly above this current. 0 = off",
                () -> Intake.TRANSFER_CURRENT_LIMIT_AMPS, v -> Intake.TRANSFER_CURRENT_LIMIT_AMPS = v);
        num(g, "Pause below frac", 0.05, 0, 1, "Transfer waits when the windmill is below this fraction of speed",
                () -> Intake.TRANSFER_PAUSE_BELOW, v -> Intake.TRANSFER_PAUSE_BELOW = v);
        num(g, "Resume above frac", 0.05, 0, 1, "...and resumes above this fraction",
                () -> Intake.TRANSFER_RESUME_ABOVE, v -> Intake.TRANSFER_RESUME_ABOVE = v);
        num(g, "Unjam after s", 0.05, 0, 3, "Back up after the windmill is bogged this long. 0 = off",
                () -> Intake.UNJAM_AFTER_SEC, v -> Intake.UNJAM_AFTER_SEC = v);
        num(g, "Unjam reverse s", 0.02, 0, 1, "How long each back-up lasts",
                () -> Intake.UNJAM_REVERSE_SEC, v -> Intake.UNJAM_REVERSE_SEC = v);
        integer(g, "Unjam tries", 0, 10, "Back-ups in a row before showing FEED JAMMED",
                () -> Intake.UNJAM_MAX_TRIES, v -> Intake.UNJAM_MAX_TRIES = (int) v);

        // ---------------- Match ----------------
        g = "Match";
        num(g, "Auto length s", 1, 5, 60, "Autonomous stops itself this long after Play",
                () -> MatchTime.AUTO_SEC, v -> MatchTime.AUTO_SEC = v);
        num(g, "TeleOp length s", 5, 0, 600, "TeleOp stops itself this long after Play (0 = never, for practice)",
                () -> MatchTime.TELEOP_SEC, v -> MatchTime.TELEOP_SEC = v);
        num(g, "First warning s left", 1, -1, 120, "Slow white LED pulse (TeleOp), -1 = off",
                () -> MatchTime.FIRST_WARNING_LEFT_SEC, v -> MatchTime.FIRST_WARNING_LEFT_SEC = v);
        num(g, "Last warning s left", 1, -1, 60, "Fast white LED strobe (auto + TeleOp), -1 = off",
                () -> MatchTime.LAST_WARNING_LEFT_SEC, v -> MatchTime.LAST_WARNING_LEFT_SEC = v);

        // ---------------- Flower arm ----------------
        g = "Flower arm";
        bool(g, "Enabled", "Master switch for the flower arm servo (TeleOp and auto)",
                () -> FlowerArm.ENABLED, v -> FlowerArm.ENABLED = v);
        num(g, "Up direction", 2, -1, 1, "+1 or -1: servo direction that moves the arm UP (flip if up goes down)",
                () -> FlowerArm.DIRECTION, v -> FlowerArm.DIRECTION = v >= 0 ? 1 : -1);
        num(g, "Up power", 0.05, 0.05, FlowerArm.MAX_SAFE_POWER, "Continuous servo speed going up (stops at touchUp)",
                () -> FlowerArm.UP_POWER, v -> FlowerArm.UP_POWER = v);
        num(g, "Down power", 0.05, 0.05, FlowerArm.MAX_SAFE_POWER, "Continuous servo speed going down (stops at touchDown)",
                () -> FlowerArm.DOWN_POWER, v -> FlowerArm.DOWN_POWER = v);
        num(g, "Max run s", 0.5, 1, 10, "Keeps going until its switch; stops (fault) after this long (default 5)",
                () -> FlowerArm.MAX_TRAVEL_SEC, v -> FlowerArm.MAX_TRAVEL_SEC = v);
        num(g, "Hold power", 0.02, 0, 0.2, "Gentle push into the switch while resting (0 = off)",
                () -> FlowerArm.HOLD_POWER, v -> FlowerArm.HOLD_POWER = v);

        // ---------------- Camera ----------------
        g = "Camera";
        bool(g, "Camera enabled", "Limelight aim, speed from distance, tip detection",
                () -> ShotConfig.AUTO_SHOT_ENABLED, v -> ShotConfig.AUTO_SHOT_ENABLED = v);
        bool(g, "Auto speed", "Speed from the shot table while AIM is held",
                () -> ShotConfig.AUTO_SPEED_ENABLED, v -> ShotConfig.AUTO_SPEED_ENABLED = v);
        bool(g, "Aim gate", "While AIM is held, feed only when aimed",
                () -> ShotConfig.AIM_GATE_ENABLED, v -> ShotConfig.AIM_GATE_ENABLED = v);
        integer(g, "Pipeline", 0, 9, "Limelight AprilTag pipeline slot",
                () -> ShotConfig.APRILTAG_PIPELINE, v -> ShotConfig.APRILTAG_PIPELINE = (int) v);
        integer(g, "Blue first tag", 0, 600, "Lowest tag id of the blue baskets",
                () -> ShotConfig.BLUE_FIRST_TAG, v -> ShotConfig.BLUE_FIRST_TAG = (int) v);
        integer(g, "Red first tag", 0, 600, "Lowest tag id of the red baskets",
                () -> ShotConfig.RED_FIRST_TAG, v -> ShotConfig.RED_FIRST_TAG = (int) v);
        num(g, "Pitch deg", 0.5, -90, 90, "Camera tilt above level",
                () -> LimelightMount.PITCH_DEGREES, v -> LimelightMount.PITCH_DEGREES = v);
        num(g, "Forward in", 0.25, -30, 30, "Robot centre forward to the lens (inches)",
                () -> LimelightMount.FORWARD_METERS / .0254, v -> LimelightMount.FORWARD_METERS = v * .0254);
        num(g, "Right in", 0.25, -30, 30, "Lens right of the centre line (inches, left is negative)",
                () -> LimelightMount.RIGHT_METERS / .0254, v -> LimelightMount.RIGHT_METERS = v * .0254);
        num(g, "Height in", 0.25, 0, 40, "Floor to lens centre (inches)",
                () -> LimelightMount.HEIGHT_METERS / .0254, v -> LimelightMount.HEIGHT_METERS = v * .0254);

        g = "Aim";
        num(g, "Aim kP", 0.001, 0, 1, "Tune FIRST with kD = 0: raise until it overshoots, then back off",
                () -> ShotConfig.AIM_KP, v -> ShotConfig.AIM_KP = v);
        num(g, "Aim kD", 0.0001, 0, 1, "Then add tiny steps of damping. Too much = jitter",
                () -> ShotConfig.AIM_KD, v -> ShotConfig.AIM_KD = v);
        num(g, "Aim direction", 2, -1, 1, "+1 or -1. Flip if AIM turns away from the basket",
                () -> ShotConfig.AIM_DIRECTION, v -> ShotConfig.AIM_DIRECTION = v >= 0 ? 1 : -1);
        num(g, "Aim max power", 0.05, 0.05, 1, "Most turn power AIM uses (video: 0.4)",
                () -> ShotConfig.AIM_MAX_POWER, v -> ShotConfig.AIM_MAX_POWER = v);
        num(g, "Aim min power", 0.01, 0, 0.3, "Least turn power outside the stop angle (0 = off). Raise if it stalls short",
                () -> ShotConfig.AIM_MIN_POWER, v -> ShotConfig.AIM_MIN_POWER = v);
        num(g, "Aim stop deg", 0.1, 0, 10, "Stops turning inside this error. Raise if it buzzes",
                () -> ShotConfig.AIM_STOP_DEG, v -> ShotConfig.AIM_STOP_DEG = v);
        num(g, "Fire tolerance deg", 0.25, 0, 20, "Feeds only inside this aim error while AIM is held",
                () -> ShotConfig.AIM_FIRE_TOLERANCE_DEG, v -> ShotConfig.AIM_FIRE_TOLERANCE_DEG = v);
        num(g, "Auto speed step", 5, 1, 200, "Table speed is rounded to this (ticks/sec)",
                () -> ShotConfig.AUTO_SPEED_STEP, v -> ShotConfig.AUTO_SPEED_STEP = v);
        num(g, "Min shot dist in", 1, 0, 200, "LEDs: closer than this = cannot score from here",
                () -> ShotConfig.MIN_SHOT_DISTANCE_IN, v -> ShotConfig.MIN_SHOT_DISTANCE_IN = v);
        num(g, "Max shot dist in", 1, 0, 300, "LEDs: farther than this = cannot score from here",
                () -> ShotConfig.MAX_SHOT_DISTANCE_IN, v -> ShotConfig.MAX_SHOT_DISTANCE_IN = v);

        g = "LEDs";
        bool(g, "LEDs enabled", "Blinkin \"led\" status colours",
                () -> Leds.ENABLED, v -> Leds.ENABLED = v);

        g = "Aim point";
        num(g, "Height in opening", 0.05, 0, 1, "0 = bottom lip, 1 = top of the CELL opening. Tune for our shooter",
                () -> ShotConfig.AIM_HEIGHT_FRACTION, v -> ShotConfig.AIM_HEIGHT_FRACTION = v);
        num(g, "Depth into cell in", 0.5, -12, 12, "Aim this far inside past the opening. Normally 0",
                () -> ShotConfig.AIM_DEPTH_IN, v -> ShotConfig.AIM_DEPTH_IN = v);
        num(g, "Cluster to opening in", 0.1, 0, 20, "Manual: tag cluster centre to opening (7.19). Tape-check",
                () -> ShotConfig.CLUSTER_TO_OPENING_IN, v -> ShotConfig.CLUSTER_TO_OPENING_IN = v);
        num(g, "Opening height in", 0.1, 0, 30, "Manual: opening bottom lip to top (14)",
                () -> ShotConfig.OPENING_HEIGHT_IN, v -> ShotConfig.OPENING_HEIGHT_IN = v);
        num(g, "Hive tilt deg", 0.5, 0, 60, "Manual: HIVE resting tilt (30)",
                () -> ShotConfig.HIVE_TILT_DEG, v -> ShotConfig.HIVE_TILT_DEG = v);

        g = "Shot model";
        bool(g, "Use shot model", "Speed from physics + table rows. OFF = table only",
                () -> ShotConfig.USE_SHOT_MODEL, v -> ShotConfig.USE_SHOT_MODEL = v);
        num(g, "Launch angle deg", 0.5, 5, 85, "MEASURE: ball exit angle above level",
                () -> ShotConfig.LAUNCH_ANGLE_DEG, v -> ShotConfig.LAUNCH_ANGLE_DEG = v);
        num(g, "Exit height in", 0.25, 0, 40, "MEASURE: ball exit point above the floor",
                () -> ShotConfig.EXIT_HEIGHT_IN, v -> ShotConfig.EXIT_HEIGHT_IN = v);
        num(g, "Exit forward in", 0.25, -20, 20, "MEASURE: exit point in front of robot centre",
                () -> ShotConfig.EXIT_FORWARD_IN, v -> ShotConfig.EXIT_FORWARD_IN = v);
        num(g, "Ticks per in/s", 0.1, 0.1, 50, "Scale: flywheel ticks/sec per ball in/s (set by G2 RS in builder)",
                () -> ShotConfig.TICKS_PER_IN_PER_SEC, v -> ShotConfig.TICKS_PER_IN_PER_SEC = v);

        // ---------------- Position hold ----------------
        g = "Position hold";
        num(g, "Stiffness", 0.25, 0, 20, "How hard the robot resists pushes. Lower if it shakes",
                () -> Constants.TELEOP_HOLD_STIFFNESS, v -> Constants.TELEOP_HOLD_STIFFNESS = v);
        num(g, "Integral KI", 0.25, 0, 20, "Walks the robot back to its spot",
                () -> TeleOpPositionHold.HOLD_KI, v -> TeleOpPositionHold.HOLD_KI = v);
        num(g, "Damping KD s", 0.01, 0, 1, "Calms shaking from the stiff gains",
                () -> TeleOpPositionHold.HOLD_KD_SECONDS, v -> TeleOpPositionHold.HOLD_KD_SECONDS = v);
        num(g, "Max hold power", 0.05, 0, 1, "Most power the hold may use",
                () -> TeleOpPositionHold.MAX_HOLD_POWER, v -> TeleOpPositionHold.MAX_HOLD_POWER = v);

        // ---------------- Autonomous ----------------
        g = "Auto assumptions";
        bool(g, "Swap start walls", "ON = UP side starts at the Basket 2 wall mark",
                () -> AutoConfig.SWAP_START_WALLS, v -> AutoConfig.SWAP_START_WALLS = v);
        integer(g, "Flower for B1 start", 1, 2, "Flower used after a Basket 1 start",
                () -> AutoConfig.FLOWER_FOR_BASKET1_START, v -> AutoConfig.FLOWER_FOR_BASKET1_START = (int) v);
        integer(g, "Flower for B2 start", 1, 2, "Flower used after a Basket 2 start",
                () -> AutoConfig.FLOWER_FOR_BASKET2_START, v -> AutoConfig.FLOWER_FOR_BASKET2_START = (int) v);
        bool(g, "Flower back first", "ON = robot reverses into the flower",
                () -> AutoConfig.FLOWER_DRIVE_IN_BACKWARDS, v -> AutoConfig.FLOWER_DRIVE_IN_BACKWARDS = v);
        bool(g, "Arm down before in", "OFF = lower the arm once against the flower",
                () -> AutoConfig.FLOWER_ARM_DOWN_BEFORE_DRIVE_IN, v -> AutoConfig.FLOWER_ARM_DOWN_BEFORE_DRIVE_IN = v);
        bool(g, "Intake driving in", "Intake runs while driving into the flower",
                () -> AutoConfig.FLOWER_INTAKE_WHILE_DRIVING_IN, v -> AutoConfig.FLOWER_INTAKE_WHILE_DRIVING_IN = v);
        bool(g, "Intake backing out", "Intake runs while backing out",
                () -> AutoConfig.FLOWER_INTAKE_WHILE_BACKING_OUT, v -> AutoConfig.FLOWER_INTAKE_WHILE_BACKING_OUT = v);
        num(g, "Intake extra s", 0.25, 0, 5, "Intake time after backing out",
                () -> AutoConfig.FLOWER_INTAKE_EXTRA_SEC, v -> AutoConfig.FLOWER_INTAKE_EXTRA_SEC = v);
        bool(g, "Spin while waiting", "DOWN side keeps the flywheel spinning during the wait",
                () -> AutoConfig.DOWN_SPIN_FLYWHEEL_WHILE_WAITING, v -> AutoConfig.DOWN_SPIN_FLYWHEEL_WHILE_WAITING = v);
        for (int i = 0; i < 3; i++) {
            final int k = i;
            String[] n = {"x m", "y m", "facing deg"};
            num(g, "Flower 1 " + n[i], i == 2 ? 5 : 0.01, -360, 360, "Red flower 1 position (blue is mirrored)",
                    () -> AutoConfig.FLOWER_1[k], v -> AutoConfig.FLOWER_1[k] = v);
        }
        for (int i = 0; i < 3; i++) {
            final int k = i;
            String[] n = {"x m", "y m", "facing deg"};
            num(g, "Flower 2 " + n[i], i == 2 ? 5 : 0.01, -360, 360, "Red flower 2 position (blue is mirrored)",
                    () -> AutoConfig.FLOWER_2[k], v -> AutoConfig.FLOWER_2[k] = v);
        }

        g = "Auto shooting";
        integer(g, "Balls", 0, 10, "Balls loaded before the match",
                () -> AutoConfig.BALLS, v -> AutoConfig.BALLS = (int) v);
        num(g, "UP shot speed", 25, 0, 2400, "Flywheel speed from the UP shot mark",
                () -> AutoConfig.UP_SHOT_SPEED, v -> AutoConfig.UP_SHOT_SPEED = v);
        num(g, "DOWN shot speed", 25, 0, 2400, "Flywheel speed from the DOWN shot mark",
                () -> AutoConfig.DOWN_SHOT_SPEED, v -> AutoConfig.DOWN_SHOT_SPEED = v);
        num(g, "Shoot budget s", 0.5, 0, 30, "Longest time spent shooting (worst 4-ball time + 1.5)",
                () -> AutoConfig.SHOOT_BUDGET_SEC, v -> AutoConfig.SHOOT_BUDGET_SEC = v);
        num(g, "No shot give up s", 0.25, 0, 10, "Feed open this long with no ball: assume empty",
                () -> AutoConfig.NO_SHOT_GIVE_UP_SEC, v -> AutoConfig.NO_SHOT_GIVE_UP_SEC = v);
        num(g, "After last shot s", 0.05, 0, 3, "Flywheel keeps running after the last ball",
                () -> AutoConfig.AFTER_LAST_SHOT_SEC, v -> AutoConfig.AFTER_LAST_SHOT_SEC = v);
        num(g, "On mark in", 0.25, 0, 24, "Feeding allowed within this distance of the shot mark",
                () -> AutoConfig.ON_MARK_INCHES, v -> AutoConfig.ON_MARK_INCHES = v);
        num(g, "On mark deg", 0.5, 0, 45, "...and this angle",
                () -> AutoConfig.ON_MARK_DEGREES, v -> AutoConfig.ON_MARK_DEGREES = v);
        num(g, "Re-centre after s", 0.25, 0.25, 5, "Off the shot mark this long = drive back onto it",
                () -> AutoConfig.ON_MARK_RECENTRE_SEC, v -> AutoConfig.ON_MARK_RECENTRE_SEC = v);
        num(g, "Re-centres before relax", 1, 0, 20, "Then the looser on-mark window below is accepted",
                () -> AutoConfig.ON_MARK_RECENTRES_BEFORE_RELAX, v -> AutoConfig.ON_MARK_RECENTRES_BEFORE_RELAX = (int) Math.round(v));
        num(g, "Relaxed on-mark in", 0.5, 1, 24, "Looser position window after the re-centres",
                () -> AutoConfig.ON_MARK_RELAXED_INCHES, v -> AutoConfig.ON_MARK_RELAXED_INCHES = v);
        num(g, "Relaxed on-mark deg", 1, 1, 45, "Looser heading window after the re-centres",
                () -> AutoConfig.ON_MARK_RELAXED_DEGREES, v -> AutoConfig.ON_MARK_RELAXED_DEGREES = v);

        g = "Auto camera";
        bool(g, "Use camera", "OFF = autonomous never uses the Limelight",
                () -> AutoConfig.USE_CAMERA, v -> AutoConfig.USE_CAMERA = v);
        num(g, "Aim max deg", 0.5, 0, 45, "Camera may turn the robot at most this far",
                () -> AutoConfig.CAMERA_AIM_MAX_DEG, v -> AutoConfig.CAMERA_AIM_MAX_DEG = v);
        num(g, "Turn sign", 2, -1, 1, "+1 or -1. Flip if the camera turns the robot away",
                () -> AutoConfig.CAMERA_TURN_SIGN, v -> AutoConfig.CAMERA_TURN_SIGN = v >= 0 ? 1 : -1);
        num(g, "Camera wait s", 0.25, 0, 5, "Wait for the camera on arrival",
                () -> AutoConfig.CAMERA_WAIT_SEC, v -> AutoConfig.CAMERA_WAIT_SEC = v);
        num(g, "Aim give up s", 0.25, 0, 10, "Shoot on odometry if not lined up after this",
                () -> AutoConfig.CAMERA_AIM_GIVE_UP_SEC, v -> AutoConfig.CAMERA_AIM_GIVE_UP_SEC = v);
        num(g, "INIT check tol in", 0.5, 0.5, 24, "INIT Limelight check: allowed distance error from the start tile",
                () -> AutoConfig.INIT_CHECK_DISTANCE_TOL_IN, v -> AutoConfig.INIT_CHECK_DISTANCE_TOL_IN = v);
        bool(g, "Camera fix position", "Correct the odometry from the basket tags (step 9.6)",
                () -> AutoConfig.CAMERA_FIX_POSITION, v -> AutoConfig.CAMERA_FIX_POSITION = v);
        num(g, "Fix max in", 0.5, 0, 48, "Bigger corrections are not believed",
                () -> AutoConfig.CAMERA_FIX_MAX_IN, v -> AutoConfig.CAMERA_FIX_MAX_IN = v);
        num(g, "Fix min in", 0.1, 0, 12, "Smaller differences are ignored",
                () -> AutoConfig.CAMERA_FIX_MIN_IN, v -> AutoConfig.CAMERA_FIX_MIN_IN = v);
        integer(g, "Fix min tags", 1, 4, "Tags needed per frame",
                () -> AutoConfig.CAMERA_FIX_MIN_TAGS, v -> AutoConfig.CAMERA_FIX_MIN_TAGS = (int) v);
        integer(g, "Fix samples", 1, 50, "Still frames used",
                () -> AutoConfig.CAMERA_FIX_SAMPLES, v -> AutoConfig.CAMERA_FIX_SAMPLES = (int) v);
        num(g, "Fix spread in", 0.25, 0, 12, "Frames must agree within this",
                () -> AutoConfig.CAMERA_FIX_SPREAD_IN, v -> AutoConfig.CAMERA_FIX_SPREAD_IN = v);
        num(g, "Tag centre x m", 0.005, -2, 2, "Red: x of the middle of the tag row",
                () -> AutoConfig.BASKET_TAG_CENTER_X_M, v -> AutoConfig.BASKET_TAG_CENTER_X_M = v);
        num(g, "Tag row y m", 0.005, -2, 2, "Distance of the tag row from the centre line",
                () -> AutoConfig.BASKET_TAG_Y_UP_M, v -> AutoConfig.BASKET_TAG_Y_UP_M = v);
        for (int i = 0; i < AutoConfig.BASKET_TAG_OFFSETS_IN.length; i++) {
            final int k = i;
            num(g, "Tag " + (i + 1) + " offset in", 0.25, -30, 30, "Tag position along the row from its middle",
                    () -> AutoConfig.BASKET_TAG_OFFSETS_IN[k], v -> AutoConfig.BASKET_TAG_OFFSETS_IN[k] = v);
        }

        g = "Auto DOWN side";
        num(g, "Max wait s", 0.5, 0, 30, "Stop waiting for the tip at this time",
                () -> AutoConfig.DOWN_MAX_WAIT_SEC, v -> AutoConfig.DOWN_MAX_WAIT_SEC = v);
        num(g, "Basket up tag in", 0.5, 0, 100, "Tags higher than this = basket is UP",
                () -> AutoConfig.BASKET_UP_TAG_HEIGHT_IN, v -> AutoConfig.BASKET_UP_TAG_HEIGHT_IN = v);
        num(g, "Tip confirm s", 0.05, 0, 3, "Basket must look UP this long",
                () -> AutoConfig.TIP_CONFIRM_SEC, v -> AutoConfig.TIP_CONFIRM_SEC = v);
        num(g, "No camera shoot at s", 0.5, -1, 30, "Shoot at this time without a camera (-1 = never)",
                () -> AutoConfig.DOWN_NO_CAMERA_SHOOT_AT_SEC, v -> AutoConfig.DOWN_NO_CAMERA_SHOOT_AT_SEC = v);
        num(g, "Min shoot s", 0.5, 0, 30, "Do not start shooting with less time than this",
                () -> AutoConfig.MIN_SHOOT_SEC, v -> AutoConfig.MIN_SHOOT_SEC = v);

        g = "Auto flower";
        num(g, "Standoff in", 0.5, 0, 48, "Stop this far back to lower the arm",
                () -> AutoConfig.FLOWER_STANDOFF_IN, v -> AutoConfig.FLOWER_STANDOFF_IN = v);
        num(g, "Arm down s", 0.1, 0.3, 10, "Longest wait for touchDown before skipping the flower (Flower Arm Test sets it)",
                () -> AutoConfig.FLOWER_ARM_DOWN_SEC, v -> AutoConfig.FLOWER_ARM_DOWN_SEC = v);
        num(g, "Arm up s", 0.1, 0.3, 10, "Longest wait for touchUp before driving away (Flower Arm Test sets it)",
                () -> AutoConfig.FLOWER_ARM_UP_SEC, v -> AutoConfig.FLOWER_ARM_UP_SEC = v);
        num(g, "Push m per s", 0.05, 0.05, 2, "Speed into the flower and back out. Lower if it bounces",
                () -> AutoConfig.FLOWER_PUSH_M_PER_S, v -> AutoConfig.FLOWER_PUSH_M_PER_S = v);
        num(g, "Collect s", 0.25, 0, 10, "Time at the flower with the intake running",
                () -> AutoConfig.FLOWER_COLLECT_SEC, v -> AutoConfig.FLOWER_COLLECT_SEC = v);

        g = "Auto timing";
        num(g, "Park by s", 0.1, 0, 30, "Must be parked by this time",
                () -> AutoConfig.PARK_BY_SEC, v -> AutoConfig.PARK_BY_SEC = v);
        num(g, "Hard stop s", 0.1, 0, 30, "Everything off at this time",
                () -> AutoConfig.HARD_STOP_SEC, v -> AutoConfig.HARD_STOP_SEC = v);
        num(g, "Park margin s", 0.1, 0, 5, "Extra time on top of the drive to the slot",
                () -> AutoConfig.PARK_MARGIN_SEC, v -> AutoConfig.PARK_MARGIN_SEC = v);
        num(g, "Leg late s", 0.5, 0, 15, "A slow drive leg accepts a wider arrival window after this",
                () -> AutoConfig.LEG_LATE_SEC, v -> AutoConfig.LEG_LATE_SEC = v);
    }

    private interface BoolGet { boolean get(); }
    private interface BoolSet { void set(boolean v); }

    private static void num(String g, String label, double step, double min, double max, String help,
                            DoubleSupplier get, DoubleConsumer set) {
        add(new Setting(g, label, Kind.NUMBER, step, min, max, help, get, set));
    }

    private static void integer(String g, String label, double min, double max, String help,
                                DoubleSupplier get, DoubleConsumer set) {
        add(new Setting(g, label, Kind.INT, 1, min, max, help, get, set));
    }

    private static void bool(String g, String label, String help, BoolGet get, BoolSet set) {
        add(new Setting(g, label, Kind.BOOL, 1, 0, 1, help, () -> get.get() ? 1 : 0, v -> set.set(v >= 0.5)));
    }

    private static void add(Setting s) {
        ALL.add(s);
        if (!GROUPS.contains(s.group)) GROUPS.add(s.group);
    }

    // =====================================================================
    // Public API
    // =====================================================================

    public static List<String> groups() { return Collections.unmodifiableList(GROUPS); }

    public static List<Setting> inGroup(String group) {
        List<Setting> out = new ArrayList<>();
        for (Setting s : ALL) if (s.group.equals(group)) out.add(s);
        return out;
    }

    public static Setting find(String group, String label) {
        for (Setting s : ALL) if (s.group.equals(group) && s.label.equals(label)) return s;
        return null;
    }

    /** Reads the saved file once per app run. Every subsystem init calls this; only the first call does work. */
    public static synchronized void ensureLoaded() {
        if (!loaded) reload();
    }

    /** Puts every setting back to the code default, then applies the saved file (drops unsaved changes). */
    public static synchronized void reload() {
        loaded = true;
        for (Setting s : ALL) s.set(s.codeDefault);
        ShotConfig.SHOT_TABLE = copyTable(codeDefaultTable);
        File f = file();
        if (f.exists()) {
            Properties p = new Properties();
            try (InputStream in = new FileInputStream(f)) {
                p.load(in);
                for (Setting s : ALL) {
                    String v = p.getProperty(s.key());
                    if (v == null) continue;
                    try { s.set(Double.parseDouble(v.trim())); } catch (NumberFormatException ignored) { }
                }
                String table = p.getProperty(TABLE_KEY);
                if (table != null) ShotConfig.SHOT_TABLE = parseTable(table);
                lastProblem = "";
            } catch (Exception e) {
                lastProblem = "could not read " + f + ": " + e.getMessage();
            }
        }
        // The simulator always runs with the optional parts on.
        if (InSimulator.YES) {
            FlowerArm.ENABLED = true;
            ShotConfig.AUTO_SHOT_ENABLED = true;
        }
    }

    /** Writes every setting to the file. @return true if saved. */
    public static synchronized boolean save() {
        Properties p = new Properties();
        for (Setting s : ALL) p.setProperty(s.key(), Double.toString(s.get()));
        p.setProperty(TABLE_KEY, tableText(ShotConfig.SHOT_TABLE));
        File f = file();
        try {
            File dir = f.getParentFile();
            if (dir != null && !dir.exists()) dir.mkdirs();
            try (OutputStream out = new FileOutputStream(f)) {
                p.store(out, "BIOBuzz robot settings. Edit with the Robot Settings OpMode.");
            }
            lastProblem = "";
            return true;
        } catch (Exception e) {
            lastProblem = "could not save " + f + ": " + e.getMessage();
            return false;
        }
    }

    /** Puts every setting and the shot table back to the code defaults and saves. */
    public static synchronized boolean resetAllToCode() {
        for (Setting s : ALL) s.set(s.codeDefault);
        ShotConfig.SHOT_TABLE = copyTable(codeDefaultTable);
        return save();
    }

    public static String problem() { return lastProblem; }

    public static File file() {
        File robot = new File("/sdcard/FIRST");
        File dir = robot.isDirectory() ? robot : new File(System.getProperty("user.home", "."));
        return new File(dir, "biobuzz-settings.properties");
    }

    // =====================================================================
    // Shot table (distance inches -> flywheel ticks/sec)
    // =====================================================================
    private static final String TABLE_KEY = "ShotTable.rows";

    /** Adds a row, replacing any row within 1 inch of the same distance, and saves. */
    public static synchronized boolean addShotRow(double distanceIn, double speed) {
        List<double[]> rows = new ArrayList<>();
        for (double[] r : ShotConfig.SHOT_TABLE) if (Math.abs(r[0] - distanceIn) >= 1.0) rows.add(r);
        rows.add(new double[]{Math.round(distanceIn * 10) / 10.0, Math.round(speed)});
        rows.sort((a, b) -> Double.compare(a[0], b[0]));
        ShotConfig.SHOT_TABLE = rows.toArray(new double[0][]);
        return save();
    }

    /** Removes the row nearest to this distance and saves. @return the removed row, or null if the table is empty. */
    public static synchronized double[] removeNearestShotRow(double distanceIn) {
        double[][] t = ShotConfig.SHOT_TABLE;
        if (t.length == 0) return null;
        int best = 0;
        if (!Double.isNaN(distanceIn)) {
            for (int i = 1; i < t.length; i++) {
                if (Math.abs(t[i][0] - distanceIn) < Math.abs(t[best][0] - distanceIn)) best = i;
            }
        } else {
            best = t.length - 1;
        }
        double[] removed = t[best];
        List<double[]> rows = new ArrayList<>();
        for (int i = 0; i < t.length; i++) if (i != best) rows.add(t[i]);
        ShotConfig.SHOT_TABLE = rows.toArray(new double[0][]);
        save();
        return removed;
    }

    public static synchronized void clearShotTable() {
        ShotConfig.SHOT_TABLE = new double[0][];
        save();
    }

    public static String tableText(double[][] t) {
        StringBuilder b = new StringBuilder();
        for (double[] r : t) {
            if (r == null || r.length < 2) continue;
            if (b.length() > 0) b.append(';');
            b.append(trim(r[0])).append(':').append(trim(r[1]));
        }
        return b.toString();
    }

    private static double[][] parseTable(String text) {
        List<double[]> rows = new ArrayList<>();
        for (String part : text.split(";")) {
            String[] dv = part.trim().split(":");
            if (dv.length != 2) continue;
            try {
                rows.add(new double[]{Double.parseDouble(dv[0].trim()), Double.parseDouble(dv[1].trim())});
            } catch (NumberFormatException ignored) { }
        }
        rows.sort((a, b) -> Double.compare(a[0], b[0]));
        return rows.toArray(new double[0][]);
    }

    private static double[][] copyTable(double[][] t) {
        if (t == null) return new double[0][];
        double[][] c = new double[t.length][];
        for (int i = 0; i < t.length; i++) c[i] = t[i] == null ? new double[0] : t[i].clone();
        return c;
    }

    /** Short number text without trailing zeros: 0.000480, 1500, 0.35. */
    public static String trim(double v) {
        if (v == Math.rint(v) && Math.abs(v) < 1e9) return String.format(Locale.US, "%d", (long) v);
        String s = String.format(Locale.US, "%.7f", v);
        s = s.replaceAll("0+$", "");
        return s.endsWith(".") ? s.substring(0, s.length() - 1) : s;
    }
}
