package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * REV Blinkin LED driver, configured as "led" (a Servo port on the hub, type "REV Blinkin LED Driver").
 *
 * <p>One object per OpMode: {@link #init}, then one of the {@code show...} / {@link #set} calls every loop.
 * A missing or unplugged Blinkin never stops an OpMode: every call just does nothing. The pattern is only sent
 * when it changes, so calling it every loop costs nothing.
 *
 * <h3>What the colours mean in TeleOp (highest priority first)</h3>
 * <table>
 *   <tr><td>flashing GOLD</td><td>feed is jammed - eject with Back</td></tr>
 *   <tr><td>fast WHITE strobe</td><td>10 seconds left (MatchTime.LAST_WARNING_LEFT_SEC), for 2 s; also in autonomous</td></tr>
 *   <tr><td>slow WHITE pulse</td><td>30 seconds left (MatchTime.FIRST_WARNING_LEFT_SEC), for 2 s</td></tr>
 *   <tr><td>GREEN</td><td>FIRE NOW: basket in range, pointed at it and the flywheel is at speed</td></tr>
 *   <tr><td>solid BLUE / RED</td><td>our basket is in view and we CAN shoot from here (our alliance colour)</td></tr>
 *   <tr><td>BLUE / RED heartbeat</td><td>our basket is in view but NOT shootable from here (too close / too far /
 *       basket down)</td></tr>
 *   <tr><td>YELLOW</td><td>flywheel spinning up (no basket in view)</td></tr>
 *   <tr><td>LIME</td><td>flywheel at speed (no basket in view / no camera)</td></tr>
 *   <tr><td>VIOLET</td><td>flower arm is down</td></tr>
 *   <tr><td>dim BLUE / RED breathing</td><td>idle (shows the alliance picked)</td></tr>
 * </table>
 * During INIT: alliance colour breathing; it alternates with flashing gold if something is missing (camera...).
 */
public class Leds {
    public static final String DEVICE_NAME = "led";

    /** Master switch. false = the Blinkin is never touched. */
    public static boolean ENABLED = true;

    // Warning times are in MatchTime (Robot Settings > Match).

    // ---- Patterns (change the look here) ----
    public static BlinkinPattern FAULT = BlinkinPattern.STROBE_GOLD;
    /** 30 s left: slow white pulse. 10 s left: fast white strobe. */
    public static BlinkinPattern FIRST_WARNING = BlinkinPattern.HEARTBEAT_WHITE;
    public static BlinkinPattern LAST_WARNING = BlinkinPattern.STROBE_WHITE;
    public static BlinkinPattern FIRE_NOW = BlinkinPattern.GREEN;
    public static BlinkinPattern SPINNING_UP = BlinkinPattern.YELLOW;
    public static BlinkinPattern FLYWHEEL_READY = BlinkinPattern.LIME;
    public static BlinkinPattern ARM_DOWN = BlinkinPattern.VIOLET;
    public static BlinkinPattern DONE_OK = BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

    private RevBlinkinLedDriver blinkin;
    private BlinkinPattern current;
    private String problem = "";

    /** Finds the Blinkin. @return true if it was found. */
    public boolean init(HardwareMap hardwareMap) {
        blinkin = null;
        current = null;
        if (!ENABLED) { problem = "off (Leds.ENABLED)"; return false; }
        try {
            blinkin = hardwareMap.get(RevBlinkinLedDriver.class, DEVICE_NAME);
            problem = "";
        } catch (Throwable e) {   // missing from the configuration, or no Blinkin class (simulator)
            Log.w("Leds", "Blinkin unavailable: " + e);
            blinkin = null;
            problem = "not found - check the configuration name \"" + DEVICE_NAME + "\"";
        }
        return blinkin != null;
    }

    public boolean isAvailable() { return blinkin != null; }

    /** Sends this pattern (only if it is different from the one showing). */
    public void set(BlinkinPattern pattern) {
        if (blinkin == null || pattern == null || pattern == current) return;
        try {
            blinkin.setPattern(pattern);
            current = pattern;
        } catch (Throwable e) {
            Log.w("Leds", "setPattern failed: " + e);
        }
    }

    /** Turns the lights off (black). */
    public void off() { set(BlinkinPattern.BLACK); }

    public BlinkinPattern getPattern() { return current; }

    // =====================================================================
    // Alliance colours
    // =====================================================================

    public static BlinkinPattern solid(boolean blue) { return blue ? BlinkinPattern.BLUE : BlinkinPattern.RED; }
    public static BlinkinPattern heartbeat(boolean blue) { return blue ? BlinkinPattern.HEARTBEAT_BLUE : BlinkinPattern.HEARTBEAT_RED; }
    public static BlinkinPattern breath(boolean blue) { return blue ? BlinkinPattern.BREATH_BLUE : BlinkinPattern.BREATH_RED; }
    public static BlinkinPattern chase(boolean blue) { return blue ? BlinkinPattern.LIGHT_CHASE_BLUE : BlinkinPattern.LIGHT_CHASE_RED; }
    public static BlinkinPattern dark(boolean blue) { return blue ? BlinkinPattern.DARK_BLUE : BlinkinPattern.DARK_RED; }

    /**
     * INIT: alliance colour breathing. If {@code allGood} is false it alternates every half second with the
     * fault colour, so a missing camera or arm is seen from across the field.
     */
    public void showInit(boolean blue, boolean allGood, double nowSec) {
        boolean faultPhase = !allGood && ((long) Math.floor(nowSec * 2)) % 2 == 0;
        set(faultPhase ? FAULT : breath(blue));
    }

    /** TeleOp INIT: alliance from autoShot; flashes gold if a switched-on camera or flower arm was not found. */
    public void showInit(AutoShot autoShot, FlowerArm flowerArm, double nowSec) {
        boolean cameraOk = !ShotConfig.AUTO_SHOT_ENABLED || autoShot.isActive();
        boolean armOk = !FlowerArm.ENABLED || flowerArm.isAvailable();
        showInit(autoShot.isBlue(), cameraOk && armOk, nowSec);
    }

    // =====================================================================
    // TeleOp
    // =====================================================================

    /**
     * Picks the TeleOp pattern from the robot state. Call once per loop, after autoShot.update() and
     * shooter.update().
     *
     * @param autoShot  camera aiming (may have no camera; then only the flywheel/arm colours are used)
     * @param aimHeld   true while the AIM button is held
     * @param teleOpSec seconds since TeleOp started (for the endgame warning)
     */
    public void showTeleOp(AutoShot autoShot, Shooter shooter, Intake intake, FlowerArm flowerArm,
                           boolean aimHeld, double teleOpSec) {
        if (blinkin == null) return;
        boolean blue = autoShot.isBlue();
        BlinkinPattern p;
        if (intake.isJammed()) {
            p = FAULT;
        } else if (timeWarning(teleOpSec, MatchTime.TELEOP_SEC, true) != null) {
            p = timeWarning(teleOpSec, MatchTime.TELEOP_SEC, true);
        } else if (autoShot.isActive() && autoShot.canShootFromHere()) {
            // In range of our basket. Green once it is also aimed and the flywheel is at speed.
            p = autoShot.isAimed() && shooter.isReady() ? FIRE_NOW : solid(blue);
        } else if (autoShot.isActive() && autoShot.getTracker().isVisible()) {
            p = heartbeat(blue);   // basket seen, but not from a spot we can shoot from
        } else if (shooter.isRunning()) {
            p = shooter.isReady() ? FLYWHEEL_READY : SPINNING_UP;
        } else if (flowerArm.isDown()) {
            p = ARM_DOWN;
        } else {
            p = breath(blue);
        }
        set(p);
    }

    /** The time-warning pattern to show now, or null. */
    public static BlinkinPattern timeWarning(double elapsedSec, double lengthSec, boolean firstWarningToo) {
        int w = MatchTime.warning(elapsedSec, lengthSec, firstWarningToo);
        return w == 2 ? LAST_WARNING : w == 1 ? FIRST_WARNING : null;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("LEDs", blinkin == null ? problem : String.valueOf(current));
    }
}
