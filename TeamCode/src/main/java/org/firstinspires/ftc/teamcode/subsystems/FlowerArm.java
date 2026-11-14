package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The flower arm (the ramp): swings DOWN so driving into a flower bed lets the balls come out, and back UP for
 * normal driving.
 *
 * <p>HARDWARE
 * <ul>
 *   <li>A CONTINUOUS rotation servo named {@link #SERVO_NAME} ("flowerArm"): it has no positions, only a speed, so
 *       the arm is driven until it hits a limit switch.</li>
 *   <li>Two REV Touch Sensors (digital ports): {@link #TOUCH_UP} ("touchUp") is pressed when the arm is fully UP,
 *       {@link #TOUCH_DOWN} ("touchDown") when it is fully DOWN.</li>
 * </ul>
 *
 * <p>SAFETY RULES (all in {@link #update()}, which MUST be called every loop while the arm can move):
 * <ul>
 *   <li>Moving up stops the moment touchUp is pressed; moving down stops the moment touchDown is pressed.</li>
 *   <li>It never drives further into a switch that is already pressed (also in manual/jog mode).</li>
 *   <li>Moving up but touchDown gets pressed (or the other way round) = the direction is reversed: stop, FAULT.</li>
 *   <li>Both switches pressed at once = wiring/sensor fault: stop, FAULT.</li>
 *   <li>A move that has not reached its switch after {@link #MAX_TRAVEL_SEC} stops: FAULT (jammed, switch missed).</li>
 *   <li>Resting UP and the arm sags off touchUp (gravity, a knock): it is driven back up onto the switch.</li>
 * </ul>
 * A FAULT stops the servo and stays until the next up()/down() command. Without {@link #ENABLED} or an explicit
 * required-arm instance, the arm stays off. Missing hardware always prevents arm movement; the OpMode decides
 * whether its route can run without the ramp.
 * Test and tune it with the "Flower Arm Test" OpMode (setup step 8).
 */
public class FlowerArm {
    /** MASTER SWITCH (Robot Settings > Flower arm > Enabled). Simulator: always on. */
    public static boolean ENABLED = false || InSimulator.YES;
    public static final String SERVO_NAME = "flowerArm", TOUCH_UP = "touchUp", TOUCH_DOWN = "touchDown";

    /** +1 or -1: which servo direction moves the arm UP. Flip it in the test OpMode if "up" goes down. */
    public static double DIRECTION = -1;   // reversed on the real robot
    /**
     * HARD SPEED LIMIT: the servo is never driven faster than this (0..1), whatever the settings say. 1.0 = Robot
     * Settings may run it at full speed; the touch sensors still stop it at each end.
     */
    public static final double MAX_SAFE_POWER = 1.0;
    /** Servo speed (0..MAX_SAFE_POWER) going up and going down. Slow on purpose; down is helped by gravity. */
    public static double UP_POWER = 0.35;
    public static double DOWN_POWER = 0.25;
    /** Soft start: the speed builds up from 0 over this many seconds at the start of every move (no jerk). */
    public static double RAMP_UP_SEC = 0.3;
    /** A move that has not hit its switch after this many seconds is stopped (fault). Measure with the test OpMode. */
    public static double MAX_TRAVEL_SEC = 5.0;   // keeps going until its switch, at most 5 s
    /**
     * While resting on a switch, keep pushing gently into it at this power (0 = off). Only if the arm sags off a
     * switch on its own; most arms do not need it.
     */
    public static double HOLD_POWER = 0.0;
    /** Resting UP, off touchUp for this long = drive it back up. */
    public static double RESEAT_SEC = 0.25;

    public enum State { UNKNOWN, MOVING_UP, UP, MOVING_DOWN, DOWN, STOPPED, MANUAL, FAULT }

    private CRServo servo;
    private TouchSensor touchUp, touchDown;
    private String problem = "";
    private State state = State.UNKNOWN;
    private String fault = "";
    private final ElapsedTime moveTimer = new ElapsedTime(), offUpTimer = new ElapsedTime();
    private boolean offUp = false;
    private boolean wantDown = false;   // last command
    private double lastMoveSec = Double.NaN;

    /** Finds the servo and switches. Does NOT move the arm. */
    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, false);
    }

    /**
     * Allows an OpMode whose route requires the ramp to enable its own arm instance.
     * Keeps saved settings unchanged and still requires the servo and both switches.
     */
    public void init(HardwareMap hardwareMap, boolean requiredForThisOpMode) {
        RobotSettings.ensureLoaded();
        servo = null;
        touchUp = touchDown = null;
        state = State.UNKNOWN;
        fault = "";
        wantDown = false;
        try { touchUp = hardwareMap.tryGet(TouchSensor.class, TOUCH_UP); } catch (RuntimeException e) { touchUp = null; }
        try { touchDown = hardwareMap.tryGet(TouchSensor.class, TOUCH_DOWN); } catch (RuntimeException e) { touchDown = null; }
        if (!ENABLED && !requiredForThisOpMode) { problem = "OFF (Robot Settings > Flower arm > Enabled)"; return; }
        try { servo = hardwareMap.tryGet(CRServo.class, SERVO_NAME); } catch (RuntimeException e) { servo = null; }
        if (servo == null) problem = "continuous servo \"" + SERVO_NAME + "\" not in the configuration";
        else if (touchUp == null) problem = "touch sensor \"" + TOUCH_UP + "\" not in the configuration";
        else if (touchDown == null) problem = "touch sensor \"" + TOUCH_DOWN + "\" not in the configuration";
        else problem = "";
        if (!problem.isEmpty()) servo = null;   // never drive without both switches
        else servo.setPower(0);
        if (servo != null) state = upPressed() ? State.UP : downPressed() ? State.DOWN : State.UNKNOWN;
    }

    /** @return true when the arm can move: switched on, servo and both switches found. */
    public boolean isAvailable() { return servo != null; }

    /** Drive up until touchUp. */
    public void up() { command(false); }

    /** Drive down until touchDown. */
    public void down() { command(true); }

    public void toggle() { command(!wantDown); }

    private void command(boolean down) {
        wantDown = down;
        if (servo == null) return;
        fault = "";
        if (down ? downPressed() : upPressed()) { stopAt(down ? State.DOWN : State.UP); return; }
        state = down ? State.MOVING_DOWN : State.MOVING_UP;
        moveTimer.reset();
    }

    /** Stop where it is. */
    public void stop() {
        if (servo != null) servo.setPower(0);
        if (state == State.MOVING_UP || state == State.MOVING_DOWN || state == State.MANUAL) state = State.STOPPED;
    }

    /**
     * Manual drive for testing: + = up, - = down, at most the set powers. Never pushes into a pressed switch.
     * Call every loop; 0 stops.
     */
    public void manual(double power) {
        if (servo == null) return;
        fault = "";
        if (power > 0 && upPressed() || power < 0 && downPressed()) power = 0;
        double upMax = Math.min(UP_POWER, MAX_SAFE_POWER), downMax = Math.min(DOWN_POWER, MAX_SAFE_POWER);
        power = Math.max(-downMax, Math.min(upMax, power));
        servo.setPower(DIRECTION * power);
        state = power == 0 ? (upPressed() ? State.UP : downPressed() ? State.DOWN : State.STOPPED) : State.MANUAL;
    }

    /** THE SAFETY LOOP. Call every loop (TeleOp and autonomous, also after the program's own work is done). */
    public void update() {
        if (servo == null || state == State.MANUAL) return;
        boolean up = upPressed(), down = downPressed();
        if (up && down) { faultStop("BOTH touch sensors pressed - check the wiring / sensors"); return; }
        switch (state) {
            case MOVING_UP:
                if (up) { lastMoveSec = moveTimer.seconds(); stopAt(State.UP); }
                else if (down && moveTimer.seconds() > 2.0)
                    faultStop("going UP but touchDown is pressed - DIRECTION is reversed?");
                else if (moveTimer.seconds() > MAX_TRAVEL_SEC) faultStop("did not reach touchUp in " + MAX_TRAVEL_SEC + " s");
                else servo.setPower(DIRECTION * movingPower(UP_POWER));
                break;
            case MOVING_DOWN:
                if (down) { lastMoveSec = moveTimer.seconds(); stopAt(State.DOWN); }
                else if (up && moveTimer.seconds() > 2.0)
                    faultStop("going DOWN but touchUp is pressed - DIRECTION is reversed?");
                else if (moveTimer.seconds() > MAX_TRAVEL_SEC) faultStop("did not reach touchDown in " + MAX_TRAVEL_SEC + " s");
                else servo.setPower(-DIRECTION * movingPower(DOWN_POWER));
                break;
            case UP:
                if (up) { offUp = false; servo.setPower(DIRECTION * Math.min(HOLD_POWER, MAX_SAFE_POWER)); }
                else {
                    if (!offUp) { offUp = true; offUpTimer.reset(); }
                    servo.setPower(0);
                    if (offUpTimer.seconds() > RESEAT_SEC) { state = State.MOVING_UP; moveTimer.reset(); }   // sagged: back up
                }
                break;
            case DOWN:
                servo.setPower(down ? -DIRECTION * Math.min(HOLD_POWER, MAX_SAFE_POWER) : 0);
                break;
            default:
                servo.setPower(0);
                break;
        }
    }

    /** The set power, capped at MAX_SAFE_POWER, built up gently over RAMP_UP_SEC. */
    private double movingPower(double power) {
        double capped = Math.min(Math.abs(power), MAX_SAFE_POWER);
        double ramp = RAMP_UP_SEC <= 0 ? 1 : Math.min(1, 0.2 + moveTimer.seconds() / RAMP_UP_SEC);
        return capped * ramp;
    }

    private void stopAt(State at) {
        servo.setPower(0);
        state = at;
        offUp = false;
    }

    private void faultStop(String why) {
        servo.setPower(0);
        state = State.FAULT;
        fault = why;
    }

    private boolean upPressed() { return touchUp != null && touchUp.isPressed(); }

    private boolean downPressed() { return touchDown != null && touchDown.isPressed(); }

    /** @return true when touchUp confirms the arm is fully up. */
    public boolean isUp() { return servo != null && upPressed() && !downPressed(); }

    /** @return true when touchDown confirms the arm is fully down. */
    public boolean isAtDown() { return servo != null && downPressed() && !upPressed(); }

    /** @return true when the arm may be sticking out (available and NOT confirmed up). Drive carefully. */
    public boolean isOut() { return servo != null && !isUp(); }

    /** @return true if the last command was DOWN (for the LEDs and toggle). */
    public boolean isDown() { return wantDown; }

    public boolean isMoving() { return state == State.MOVING_UP || state == State.MOVING_DOWN || state == State.MANUAL; }

    public boolean isFaulted() { return state == State.FAULT; }

    public State getState() { return state; }

    /** Seconds the last complete move (switch to switch) took, or NaN. */
    public double getLastMoveSec() { return lastMoveSec; }

    /** Switch readings even when the arm is switched off (for checking the sensors). */
    public String switchText() {
        return "touchUp " + (touchUp == null ? "MISSING" : touchUp.isPressed() ? "PRESSED" : "open")
                + ", touchDown " + (touchDown == null ? "MISSING" : touchDown.isPressed() ? "PRESSED" : "open");
    }

    public String getStatus() {
        if (servo == null) return problem;
        if (state == State.FAULT) return "FAULT: " + fault;
        return state.toString();
    }

    public void addTelemetry(Telemetry telemetry, String buttons) {
        if (!ENABLED) return;
        telemetry.addData("Flower arm (" + buttons + ")", getStatus());
    }
}
