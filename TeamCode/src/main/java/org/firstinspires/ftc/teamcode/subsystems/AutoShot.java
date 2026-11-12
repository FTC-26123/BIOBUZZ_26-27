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
    private final ShotTable table = new ShotTable(ShotConfig.SHOT_TABLE);

    private boolean cameraReady = false;
    private boolean blue = ShotConfig.DEFAULT_ALLIANCE_BLUE;
    private boolean aimHeld = false;
    private boolean permit = true;
    private String cameraProblem = "";

    public void init(HardwareMap hardwareMap, Limelight limelight, AprilTagAlignment alignment, Shooter shooter,
                     ElapsedTime clock) {
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
    }

    /** Call from init_loop(): gamepad 1 X = blue alliance, B = red alliance. */
    public void selectAlliance(Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.xWasPressed()) blue = true;
        if (gamepad.bWasPressed()) blue = false;
        if (!ShotConfig.AUTO_SHOT_ENABLED) {
            telemetry.addLine("Camera aim/speed: OFF (ShotConfig.AUTO_SHOT_ENABLED)");
            return;
        }
        telemetry.addData("ALLIANCE (G1 X = blue, B = red)", blue ? "BLUE" : "RED");
        telemetry.addData("Camera", cameraReady ? "found" : cameraProblem);
        telemetry.addData("Shot table", table.isReady() ? table.size() + " rows" : "EMPTY - manual presets only");
    }

    public void start() {
        if (cameraReady) limelight.start();
    }

    public void stop() {
        if (cameraReady) limelight.stop();
    }

    /** @return true when camera aiming is switched on and the camera was found. */
    public boolean isActive() { return cameraReady; }

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
        if (ShotConfig.AUTO_SPEED_ENABLED && table.isReady() && tracker.isVisible() && tracker.hasDistance()
                && (aimHeld || shooter.isAutoTarget())) {
            double step = Math.max(1, ShotConfig.AUTO_SPEED_STEP);
            double speed = Math.round(table.speedFor(tracker.getDistanceInches()) / step) * step;
            if (!shooter.isAutoTarget() || Math.abs(speed - shooter.getAutoBaseSpeed()) >= step) {
                shooter.setAutoTarget(speed);
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

    public TargetTracker getTracker() { return tracker; }

    /** Driver Station lines. "Distance (in)" is the number to write down when measuring the shot table. */
    public void addTelemetry(Telemetry telemetry) {
        if (!ShotConfig.AUTO_SHOT_ENABLED) return;
        if (!cameraReady) {
            telemetry.addLine("Camera aim: " + cameraProblem);
            return;
        }
        telemetry.addLine("--------------- Camera aim (" + (blue ? "BLUE" : "RED") + ") ---------------");
        if (tracker.isVisible()) {
            telemetry.addData("Basket", "tags %d-%d, %d in view", tracker.getBasketFirstTag(),
                    tracker.getBasketFirstTag() + ShotConfig.TAGS_PER_BASKET - 1, tracker.getTagCount());
            telemetry.addData("Aim error (deg)", "%+.1f  %s", tracker.getAimErrorDeg(), isAimed() ? "AIMED" : "");
        } else {
            telemetry.addData("Basket", aimHeld ? "NO TARGET - release AIM to fire manually" : "not in view");
        }
        if (tracker.hasDistance()) telemetry.addData("Distance (in)", "%.1f", tracker.getDistanceInches());
        else telemetry.addData("Distance (in)", "unknown");
        if (!table.isReady()) {
            telemetry.addLine("Shot table EMPTY: speed stays on manual presets");
        } else if (tracker.hasDistance()) {
            telemetry.addData("Table speed", "%.0f ticks/s%s", table.speedFor(tracker.getDistanceInches()),
                    table.inRange(tracker.getDistanceInches()) ? "" : "  !! OUTSIDE TABLE RANGE !!");
        }
        if (aimHeld && !permit) telemetry.addLine("Holding fire until aimed");
    }
}
