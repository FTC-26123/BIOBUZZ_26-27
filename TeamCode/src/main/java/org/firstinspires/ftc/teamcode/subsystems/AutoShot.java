package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Ties the Limelight to aiming and shot speed for TeleOp. One object per TeleOp; the TeleOp calls
 * {@link #init}, {@link #selectAlliance} (during INIT), {@link #start}, {@link #update} (every loop) and
 * {@link #stop}.
 *
 * <p>What it does each loop, when {@link ShotConfig#AUTO_SHOT_ENABLED} is true and the camera was found:
 * <ol>
 *   <li>Reads the camera and finds our basket ({@link TargetTracker}).</li>
 *   <li>AIM: feeds the aim error to {@link AprilTagAlignment}; the TeleOp turns the robot while the AIM button
 *       is held.</li>
 *   <li>SPEED: while AIM is held and the basket is in view, the flywheel target is taken from the distance
 *       table ("AUTO" speed). It keeps following the distance for as long as the tags stay in view, and keeps
 *       the last value after that. Pressing a preset button on the gamepad goes back to manual presets until
 *       the next time AIM finds the basket.</li>
 *   <li>GATE: while AIM is held, balls are only fed when the robot is pointed at the basket. Firing without
 *       AIM held is never blocked.</li>
 * </ol>
 * If the switch is off, the camera is missing, or the table is empty, the matching part simply does nothing
 * and the manual controls work as before.
 */
public class AutoShot {
    private Limelight limelight;
    private AprilTagAlignment alignment;
    private Shooter shooter;
    private ElapsedTime clock;
    private final TargetTracker tracker = new TargetTracker();
    private ShotTable table = new ShotTable(ShotConfig.SHOT_TABLE);

    private boolean cameraReady = false;
    private boolean blue = ShotConfig.DEFAULT_ALLIANCE_BLUE;
    private boolean aimHeld = false;
    private boolean permit = true;
    private String cameraProblem = "";

    public void init(HardwareMap hardwareMap, Limelight limelight, AprilTagAlignment alignment, Shooter shooter,
                     ElapsedTime clock) {
        RobotSettings.ensureLoaded();
        reloadTable();
        this.limelight = limelight;
        this.alignment = alignment;
        this.shooter = shooter;
        this.clock = clock;
        cameraReady = false;
        if (!ShotConfig.AUTO_SHOT_ENABLED) return;
        // A missing or unplugged camera must never stop TeleOp from starting.
        cameraReady = limelight.initSafely(hardwareMap, ShotConfig.APRILTAG_PIPELINE);
        if (!cameraReady) {
            cameraProblem = "Limelight not found - check the robot configuration name \"limelight\"";
            return;
        }
        alignment.useTargetTracker(tracker);
        alignment.setRotationGains(ShotConfig.AIM_KP, ShotConfig.AIM_KD);
        alignment.setAimDirection(ShotConfig.AIM_DIRECTION);
        alignment.setAngleTolerance(ShotConfig.AIM_STOP_DEG);
        alignment.setRotationLimits(ShotConfig.AIM_MIN_POWER, ShotConfig.AIM_MAX_POWER);
    }

    private boolean allianceLocked = false;

    /** Fixes the alliance (Blue/Red TeleOp files). The X/B buttons in selectAlliance() are then ignored. */
    public void lockAlliance(boolean blueAlliance) {
        blue = blueAlliance;
        allianceLocked = true;
    }

    /** Call from init_loop(): gamepad 1 X = blue alliance, B = red alliance (unless the alliance is locked). */
    public void selectAlliance(Gamepad gamepad, Telemetry telemetry) {
        if (!allianceLocked && gamepad.xWasPressed()) blue = true;
        if (!allianceLocked && gamepad.bWasPressed()) blue = false;
        if (!ShotConfig.AUTO_SHOT_ENABLED) {
            telemetry.addLine("Camera aim/speed: OFF (ShotConfig.AUTO_SHOT_ENABLED)");
            return;
        }
        telemetry.addData(allianceLocked ? "ALLIANCE (set by this TeleOp)" : "ALLIANCE (G1 X = blue, B = red)",
                blue ? "BLUE" : "RED");
        telemetry.addData("Camera", cameraReady ? "found" : cameraProblem);
        telemetry.addData("Auto speed", (ShotConfig.USE_SHOT_MODEL ? "model" : "table") + ", "
                + table.size() + " table rows");
    }

    public void start() {
        if (cameraReady) limelight.start();
    }

    public void stop() {
        if (cameraReady) limelight.stop();
    }

    /** @return true when camera aiming is switched on and the camera was found. */
    public boolean isActive() { return cameraReady; }

    /** Re-applies the aim settings (gains, direction, stop angle, pipeline) after they were changed. */
    public void applySettings() {
        reloadTable();
        if (!cameraReady) return;
        alignment.setRotationGains(ShotConfig.AIM_KP, ShotConfig.AIM_KD);
        alignment.setAimDirection(ShotConfig.AIM_DIRECTION);
        alignment.setAngleTolerance(ShotConfig.AIM_STOP_DEG);
        alignment.setRotationLimits(ShotConfig.AIM_MIN_POWER, ShotConfig.AIM_MAX_POWER);
        limelight.setPipeline(ShotConfig.APRILTAG_PIPELINE);
    }

    /** Re-reads ShotConfig.SHOT_TABLE (after the Shot Table Builder changed it). */
    public void reloadTable() { table = new ShotTable(ShotConfig.SHOT_TABLE); }

    /**
     * Call once per loop, before aprilTagAlignment.update() and shooter.update().
     *
     * @param aimHeld true while the driver holds the AIM button
     */
    public void update(boolean aimHeld) {
        this.aimHeld = aimHeld;
        permit = true;
        if (!cameraReady) {
            shooter.setFeedPermit(true);
            return;
        }
        limelight.update();
        tracker.update(limelight.getLastResult(), blue, clock.seconds());

        // ---- shot speed from distance
        if (ShotConfig.AUTO_SPEED_ENABLED && ShotModel.available(table) && tracker.isVisible()
                && tracker.hasDistance() && (aimHeld || shooter.isAutoTarget())) {
            double raw = autoSpeed();
            if (!Double.isNaN(raw)) {
                double step = Math.max(1, ShotConfig.AUTO_SPEED_STEP);
                double speed = Math.round(raw / step) * step;
                if (!shooter.isAutoTarget() || Math.abs(speed - shooter.getAutoBaseSpeed()) >= step) {
                    shooter.setAutoTarget(speed);
                }
            }
        }

        // ---- only feed when pointed at the basket (while AIM is held)
        if (aimHeld && ShotConfig.AIM_GATE_ENABLED) {
            permit = tracker.isVisible() && Math.abs(tracker.getAimErrorDeg()) <= ShotConfig.AIM_FIRE_TOLERANCE_DEG;
        }
        shooter.setFeedPermit(permit);
    }

    /** @return true when the robot is pointed at the basket within the firing tolerance. */
    public boolean isAimed() {
        return cameraReady && tracker.isVisible()
                && Math.abs(tracker.getAimErrorDeg()) <= ShotConfig.AIM_FIRE_TOLERANCE_DEG;
    }

    /**
     * @return true when our UP basket is in view and the robot is standing somewhere it can score from:
     * between {@link ShotConfig#MIN_SHOT_DISTANCE_IN} and {@link ShotConfig#MAX_SHOT_DISTANCE_IN}, and (when a
     * speed model/table exists) the shot is physically possible from here. Says nothing about aim or flywheel.
     */
    public boolean canShootFromHere() {
        return inShootingRange(tracker, table);
    }

    /** Same check for any tracker (used by autonomous too). */
    public static boolean inShootingRange(TargetTracker tracker, ShotTable table) {
        if (!tracker.isCellUp() || !tracker.hasDistance()) return false;
        double d = tracker.getDistanceInches();
        if (d < ShotConfig.MIN_SHOT_DISTANCE_IN || d > ShotConfig.MAX_SHOT_DISTANCE_IN) return false;
        if (!ShotModel.available(table)) return true;
        return !Double.isNaN(ShotModel.speedFor(d, tracker.getAimHeightInches(), table));
    }

    /** @return true for the blue alliance (picked during INIT). */
    public boolean isBlue() { return blue; }

    public TargetTracker getTracker() { return tracker; }

    /** Flywheel speed for the current distance and aim height (model + table), NaN if unknown. */
    public double autoSpeed() {
        return ShotModel.speedFor(tracker.getDistanceInches(), tracker.getAimHeightInches(), table);
    }

    /** The table this TeleOp uses (rebuilt by {@link #reloadTable()}). */
    public ShotTable getTable() { return table; }

    /** Two Driver Station lines at most. */
    public void addTelemetry(Telemetry telemetry) {
        if (!ShotConfig.AUTO_SHOT_ENABLED) return;
        if (!cameraReady) {
            telemetry.addLine("Camera: " + cameraProblem);
            return;
        }
        if (tracker.isVisible()) {
            telemetry.addLine(String.format(java.util.Locale.US, "Aim %d-%d %s  %d tags  err %+.1f%s",
                    tracker.getBasketFirstTag(), tracker.getBasketFirstTag() + ShotConfig.TAGS_PER_BASKET - 1,
                    tracker.isCellUp() ? "UP" : "down", tracker.getTagCount(), tracker.getAimErrorDeg(),
                    isAimed() ? "  AIMED" : aimHeld && !permit ? "  (holding fire)" : ""));
        } else {
            telemetry.addLine(aimHeld ? "Aim: NO TARGET - release AIM to fire manually" : "Aim: no target in view");
        }
        if (tracker.hasDistance()) {
            double speed = autoSpeed();
            telemetry.addLine(String.format(java.util.Locale.US, "Dist %.0f in  auto speed %s",
                    tracker.getDistanceInches(), Double.isNaN(speed) ? "-" : String.format(java.util.Locale.US, "%.0f", speed)));
        }
    }
}
