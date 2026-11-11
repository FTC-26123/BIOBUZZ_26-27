package org.firstinspires.ftc.teamcode.teleop.tests;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TeleOpPositionHold;

/**
 * Tunes the drivetrain position hold (the "brick in place" behavior).
 *
 * <p>Uses the same Pedro Foresight hold calculation as {@code TeleOpPositionHold}, but with gains you can change
 * live, plus the same integral and damping the real hold adds. It never edits production code: when you are happy,
 * copy the printed values into {@code Constants.TELEOP_HOLD_STIFFNESS} and the HOLD_* constants in
 * {@code TeleOpPositionHold}.
 *
 * <p>MANUAL: A = capture this spot and start holding, B = release (motors off). Push the robot by hand or with
 * another robot; telemetry shows peak push distance, recovery time and ringing.
 * Dpad up/down = pick a setting, dpad left/right = change it, bumpers = bigger/smaller step, Y = clear stats.
 *
 * <p>AUTO: X = the robot kicks itself forward then sideways and measures the response, raising the stiffness each
 * round until it starts ringing, then backs off 15%. Clear 1 m of space around the robot first. Move any stick
 * or press B to abort.
 */
@TeleOp(name = "Tune Position Hold", group = "Tuning")
public class TeleOp_TuneHold extends OpMode {

    // Starts from the production values so the tuner behaves exactly like the real TeleOp hold.
    private static final String[] NAMES = {"Stiffness", "Integral KI", "Damping KD (s)", "Max hold power"};
    private final double[] steps = {0.25, 0.25, 0.01, 0.05};
    private double gainScale = Constants.TELEOP_HOLD_STIFFNESS;
    private double holdKi = TeleOpPositionHold.HOLD_KI;
    private double holdKd = TeleOpPositionHold.HOLD_KD_SECONDS;
    private double maxPower = TeleOpPositionHold.MAX_HOLD_POWER;
    private final TeleOpPositionHold.HoldExtras extras = new TeleOpPositionHold.HoldExtras();
    private int selected = 0;
    private double stepMultiplier = 1.0;

    private Follower follower;
    private Foresight foresight;
    private Pose target;
    private boolean holding = false;
    private double lastTime = Double.NaN;

    // Push statistics for the current disturbance
    private double peakOffset, episodePeak, lastRecovery = Double.NaN, lastPeak = Double.NaN;
    private double disturbedAt = Double.NaN, prevOffset = 0;
    private int flips = 0, lastFlips = 0, lastSign = 0;
    private double livePeak = 0;

    // Auto sweep
    private enum Auto { OFF, KICK_FWD, WAIT_FWD, KICK_STRAFE, WAIT_STRAFE, DONE }
    private Auto auto = Auto.OFF;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private int trial = 0;
    private double trialWorstPeak, trialWorstRecovery, lastGoodScale;
    private int trialMaxFlips;
    private String autoMessage = "";
    private static final double KICK_POWER = 0.5, KICK_SECONDS = 0.15, WAIT_SECONDS = 2.0;
    private static final int MAX_TRIALS = 8, RINGING_FLIPS = 6;

    // Guided tuning: 0 = hold + auto-tune, 1 = hand push test, 2 = robot push test, 3 = done
    private int phase = 0;
    private static final double SOFT_PEAK_INCHES = 3.0, SOFT_RECOVERY_SECONDS = 1.0, ROBOT_PUSH_PEAK_INCHES = 4.0;
    private static final int RING_FLIPS_WARN = 4;

    private boolean prevA, prevB, prevX, prevY, prevUp, prevDown, prevLeft, prevRight, prevLb, prevRb, prevStart;

    @Override
    public void init() {
        follower = Constants.createTeleOpHold(hardwareMap);
        rebuildForesight();
        telemetry.addLine("Ready. Press A to hold, X for auto-tune.");
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double dt = Double.isNaN(lastTime) ? 0.02 : Math.max(0.001, Math.min(0.1, now - lastTime));
        lastTime = now;

        follower.localizer.update();
        Pose pose = follower.pose();

        handleButtons(pose);
        if (stickMoved() && auto != Auto.OFF && auto != Auto.DONE) abortAuto("Aborted: stick moved");

        if (holding && target != null) {
            if (auto == Auto.KICK_FWD) kick(KICK_POWER, 0, now);
            else if (auto == Auto.KICK_STRAFE) kick(0, KICK_POWER, now);
            else hold(dt);
            trackPush(pose, now);
            runAutoStateMachine(now);
        } else {
            follower.drivetrain.stop(true);
        }
        showTelemetry(pose);
    }

    @Override
    public void stop() {
        if (follower != null) follower.drivetrain.stop(true);
    }

    // ------------------------------------------------------------------ control

    private void hold(double dt) {
        DrivePowers p = foresight.calculateHold(follower.drivetrain, target, follower.localizer.state(), true, dt);
        extras.kI = holdKi;
        extras.kD = holdKd;
        follower.drivetrain.drive(limit(extras.apply(p, dt)), false);
    }

    private void kick(double forward, double strafe, double now) {
        follower.drivetrain.drive(new DrivePowers(forward, strafe, 0), false);
        extras.reset();   // the kick is not a push to integrate
        if (stateTimer.seconds() >= KICK_SECONDS) {
            auto = (auto == Auto.KICK_FWD) ? Auto.WAIT_FWD : Auto.WAIT_STRAFE;
            stateTimer.reset();
            resetEpisode();
        }
    }

    private DrivePowers limit(DrivePowers p) {
        double sum = Math.abs(p.forward()) + Math.abs(p.strafe()) + Math.abs(p.turn());
        double scale = Math.max(1, sum / maxPower);
        return new DrivePowers(p.forward() / scale, p.strafe() / scale, p.turn() / scale);
    }

    private void rebuildForesight() {
        foresight = new Foresight(Constants.holdConfig(gainScale));
        extras.reset();
        // Reuse the already-initialized localizer and drivetrain; only the controller changes.
        follower = new Follower(follower.localizer, follower.drivetrain, foresight);
    }

    private boolean stickMoved() {
        return Math.abs(gamepad1.left_stick_x) > 0.3 || Math.abs(gamepad1.left_stick_y) > 0.3
                || Math.abs(gamepad1.right_stick_x) > 0.3;
    }

    private void handleButtons(Pose pose) {
        boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down, left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        boolean lb = gamepad1.left_bumper, rb = gamepad1.right_bumper;

        if (a && !prevA) { target = pose; holding = true; resetStats(); extras.reset(); }
        if (b && !prevB) { abortAuto("Released"); holding = false; extras.reset(); }
        if (y && !prevY) resetStats();
        boolean start = gamepad1.start;
        if (start && !prevStart && holding && (auto == Auto.OFF || auto == Auto.DONE) && phase < 3) {
            phase++;
            resetStats();
        }
        prevStart = start;
        if (x && !prevX && holding && (auto == Auto.OFF || auto == Auto.DONE)) startAuto();
        if (up && !prevUp) selected = (selected + NAMES.length - 1) % NAMES.length;
        if (down && !prevDown) selected = (selected + 1) % NAMES.length;
        if (lb && !prevLb) stepMultiplier = Math.max(0.1, stepMultiplier / 2);
        if (rb && !prevRb) stepMultiplier = Math.min(8, stepMultiplier * 2);
        if (auto == Auto.OFF || auto == Auto.DONE) {
            if (left && !prevLeft) adjust(-1);
            if (right && !prevRight) adjust(+1);
        }
        prevA = a; prevB = b; prevX = x; prevY = y;
        prevUp = up; prevDown = down; prevLeft = left; prevRight = right; prevLb = lb; prevRb = rb;
    }

    private void adjust(int dir) {
        double d = dir * steps[selected] * stepMultiplier;
        switch (selected) {
            case 0: gainScale = Math.max(0.25, gainScale + d); break;
            case 1: holdKi = Math.max(0, holdKi + d); break;
            case 2: holdKd = Math.max(0, holdKd + d); break;
            default: maxPower = Math.max(0.2, Math.min(1.0, maxPower + d)); break;
        }
        rebuildForesight();
        resetStats();   // earlier push results no longer apply to the new settings
    }

    // ------------------------------------------------------------------ measurements

    private void resetEpisode() {
        episodePeak = 0; flips = 0; lastSign = 0; disturbedAt = Double.NaN; livePeak = 0;
    }

    private void resetStats() {
        resetEpisode();
        peakOffset = 0; lastRecovery = Double.NaN; lastPeak = Double.NaN; lastFlips = 0;
    }

    private void trackPush(Pose pose, double now) {
        double offset = target.distance(pose);
        peakOffset = Math.max(peakOffset, offset);
        if (offset > 1.0 && Double.isNaN(disturbedAt)) { disturbedAt = now; episodePeak = 0; flips = 0; lastSign = 0; }
        if (!Double.isNaN(disturbedAt)) {
            episodePeak = Math.max(episodePeak, offset);
            livePeak = episodePeak;
            double delta = offset - prevOffset;
            if (Math.abs(delta) > 0.05) {
                int sign = delta > 0 ? 1 : -1;
                if (lastSign != 0 && sign != lastSign) flips++;
                lastSign = sign;
            }
            if (offset < 0.4) {
                lastRecovery = now - disturbedAt;
                lastPeak = episodePeak;
                lastFlips = flips;
                disturbedAt = Double.NaN;
            }
        }
        prevOffset = offset;
    }

    // ------------------------------------------------------------------ auto sweep

    private void startAuto() {
        trial = 0;
        lastGoodScale = gainScale;
        autoMessage = "Auto-tune running...";
        beginTrial();
    }

    private void beginTrial() {
        trial++;
        trialWorstPeak = 0; trialWorstRecovery = 0; trialMaxFlips = 0;
        resetEpisode();
        auto = Auto.KICK_FWD;
        stateTimer.reset();
    }

    private void runAutoStateMachine(double now) {
        if (auto != Auto.WAIT_FWD && auto != Auto.WAIT_STRAFE) return;
        boolean waitDone = stateTimer.seconds() >= WAIT_SECONDS;
        if (!waitDone) return;

        // Episode result: if it never recovered, disturbedAt is still set.
        double recovery = Double.isNaN(disturbedAt) ? (Double.isNaN(lastRecovery) ? 0 : lastRecovery) : WAIT_SECONDS;
        trialWorstPeak = Math.max(trialWorstPeak, Math.max(episodePeak, Double.isNaN(lastPeak) ? 0 : lastPeak));
        trialWorstRecovery = Math.max(trialWorstRecovery, recovery);
        trialMaxFlips = Math.max(trialMaxFlips, Math.max(flips, lastFlips));
        resetEpisode();
        lastRecovery = Double.NaN; lastFlips = 0; lastPeak = Double.NaN;

        if (auto == Auto.WAIT_FWD) {
            auto = Auto.KICK_STRAFE;
            stateTimer.reset();
            return;
        }
        // End of one trial (forward + strafe)
        boolean ringing = trialMaxFlips >= RINGING_FLIPS || trialWorstRecovery >= WAIT_SECONDS;
        if (ringing) {
            double ringingScale = gainScale;
            gainScale = Math.max(0.25, lastGoodScale * 0.85);
            rebuildForesight();
            finishAuto(String.format("Ringing at x%.2f. Using x%.2f (85%% of last good).", ringingScale, gainScale));
        } else if (trialWorstPeak < 0.4 || trial >= MAX_TRIALS) {
            finishAuto(String.format("Stopped at x%.2f (no ringing; peak %.2f in).", gainScale, trialWorstPeak));
        } else {
            lastGoodScale = gainScale;
            gainScale *= 1.25;
            rebuildForesight();
            beginTrial();
        }
    }

    private void finishAuto(String message) {
        auto = Auto.DONE;
        autoMessage = message;
        resetStats();
    }

    private void abortAuto(String message) {
        if (auto != Auto.OFF) {
            auto = Auto.OFF;
            autoMessage = message;
        }
    }

    // ------------------------------------------------------------------ telemetry

    /** Prints exact next-step instructions based on where the tuning is. */
    private void showGuide() {
        telemetry.addLine("=========== WHAT TO DO NEXT ===========");
        int ring = Math.max(flips, lastFlips);
        boolean stillDisplaced = !Double.isNaN(disturbedAt);
        if (!holding) {
            telemetry.addLine("STEP 1: Put the robot on open floor with 1 m free on every side.");
            telemetry.addLine("Press A on gamepad 1. The robot now holds the spot it is standing on.");
        } else if (auto != Auto.OFF && auto != Auto.DONE) {
            telemetry.addLine("STEP 2: AUTO-TUNE RUNNING (up to about 40 s). Do not touch the robot or sticks.");
            telemetry.addLine("The robot will jump a few inches forward, then sideways, and come back.");
            telemetry.addLine("Press B or move a stick only if you need to abort.");
        } else if (phase == 0) {
            if (auto == Auto.DONE) {
                telemetry.addLine("STEP 2 DONE: " + autoMessage);
                telemetry.addLine("Press START on gamepad 1 to go to the hand push test.");
            } else {
                telemetry.addLine("STEP 2: Clear 1 m around the robot. Hands off the sticks.");
                telemetry.addLine("Press X to run auto-tune. (Press START to skip it and tune by hand.)");
            }
        } else if (phase == 1) {
            telemetry.addLine("STEP 3: HAND PUSH TEST.");
            telemetry.addLine("1) Press Y to clear the numbers.");
            telemetry.addLine("2) Push the robot sideways with firm effort for 2 s, then let go.");
            telemetry.addLine("3) Wait 2 s and read 'Peak push' and 'Last recovery' below.");
            if (stillDisplaced) {
                telemetry.addLine("RESULT: still away from the spot. Wait 2 s. If it never returns, it is too soft: see TOO SOFT below.");
            }
            if (Double.isNaN(lastPeak) && !stillDisplaced) {
                telemetry.addLine("RESULT: waiting for you to push the robot...");
            } else if (!Double.isNaN(lastPeak)) {
                giveHoldAdvice(lastPeak, lastRecovery, ring, false);
            }
        } else if (phase == 2) {
            telemetry.addLine("STEP 4: ROBOT PUSH TEST (a teammate with another robot).");
            telemetry.addLine("1) Press Y to clear the numbers.");
            telemetry.addLine("2) Teammate drives their robot into the side of yours and pushes for 3 s, then backs off.");
            telemetry.addLine("3) Wait 2 s and read 'Largest push seen' below.");
            if (peakOffset < 1.0) {
                telemetry.addLine("RESULT: waiting for the push...");
            } else if (stillDisplaced) {
                telemetry.addLine("RESULT: still being pushed or has not come back yet. Wait 2 s.");
            } else {
                giveHoldAdvice(peakOffset, lastRecovery, ring, true);
            }
        } else {
            telemetry.addLine("STEP 5: DONE.");
            telemetry.addLine("Copy the values under 'Copy into the code' (below): TELEOP_HOLD_STIFFNESS in");
            telemetry.addLine("pedro/Constants.java, the HOLD_ values in TeleOpPositionHold.java. Then press STOP.");
        }
        telemetry.addLine("=======================================");
    }

    private void giveHoldAdvice(double peak, double recovery, int ring, boolean robotPush) {
        double limitPeak = robotPush ? ROBOT_PUSH_PEAK_INCHES : SOFT_PEAK_INCHES;
        boolean ringing = ring >= RING_FLIPS_WARN;
        boolean soft = peak > limitPeak || (!Double.isNaN(recovery) && recovery > SOFT_RECOVERY_SECONDS);
        if (ringing) {
            telemetry.addLine(String.format("RESULT: TOO STIFF, IT SHAKES (%d direction flips).", ring));
            telemetry.addLine("1) Press dpad UP or DOWN until the '>' arrow is on 'Damping KD (s)'.");
            telemetry.addLine("2) Press dpad RIGHT 2 times (each press = +0.01).  3) Press Y.  4) Push again.");
            telemetry.addLine("Still shaking? Put the arrow on 'Stiffness' and press dpad LEFT 2 times (-0.25 each).");
        } else if (soft) {
            telemetry.addLine(String.format("RESULT: TOO SOFT. Pushed %.1f in (limit %.1f).", peak, limitPeak));
            if (maxPower < 0.99) {
                telemetry.addLine("1) Press dpad UP or DOWN until the '>' arrow is on 'Max hold power'.");
                telemetry.addLine("2) Press dpad RIGHT 2 times (each press = +0.05, up to 1.00).  3) Press Y.  4) Push again.");
            } else {
                telemetry.addLine("1) Press dpad UP or DOWN until the '>' arrow is on 'Stiffness'.");
                telemetry.addLine("2) Press dpad RIGHT 2 times (each press = +0.25).  3) Press Y.  4) Push again.");
                telemetry.addLine("If it gives way slowly over a long push, raise 'Integral KI' instead (+0.25 each).");
                telemetry.addLine("If the wheels spin or skid while it gives way, that is grip, not tuning. Go to the next step.");
            }
        } else {
            telemetry.addLine(String.format("RESULT: PASSED (pushed %.1f in, back in %.1f s).", peak,
                    Double.isNaN(recovery) ? 0 : recovery));
            telemetry.addLine("Press START on gamepad 1 for the next step.");
        }
    }

    private void showTelemetry(Pose pose) {
        showGuide();
        telemetry.addData("State", holding ? (auto == Auto.OFF || auto == Auto.DONE ? "HOLDING" : "AUTO " + auto) : "RELEASED");
        for (int i = 0; i < NAMES.length; i++) {
            double v = i == 0 ? gainScale : i == 1 ? holdKi : i == 2 ? holdKd : maxPower;
            telemetry.addData((i == selected ? "> " : "  ") + NAMES[i], "%.3f", v);
        }
        telemetry.addData("Step size", "x%.2f (bumpers)", stepMultiplier);
        if (holding && target != null) {
            double off = target.distance(pose);
            telemetry.addData("Offset now (in)", "%.2f", off);
            telemetry.addData("Peak push, current (in)", "%.2f", livePeak);
            telemetry.addData("Largest push seen (in)", "%.2f", peakOffset);
            telemetry.addData("Last recovery (s)", Double.isNaN(lastRecovery) ? "-" : String.format("%.2f", lastRecovery));
            telemetry.addData("Ringing (direction flips)", "%d", Math.max(flips, lastFlips));
        }
        if (auto != Auto.OFF) telemetry.addData("Auto trial", "%d / %d  (%s)", trial, MAX_TRIALS, auto);
        if (!autoMessage.isEmpty()) telemetry.addData("Auto result", autoMessage);
        telemetry.addLine("--- Copy into the code if you like it ---");
        telemetry.addData("Constants.TELEOP_HOLD_STIFFNESS", "%.2f", gainScale);
        telemetry.addData("TeleOpPositionHold.HOLD_KI", "%.2f", holdKi);
        telemetry.addData("TeleOpPositionHold.HOLD_KD_SECONDS", "%.3f", holdKd);
        telemetry.addData("TeleOpPositionHold.MAX_HOLD_POWER", "%.2f", maxPower);
        telemetry.addLine("A hold | B release | X auto | Y clear | dpad pick/change | bumpers step");
        telemetry.update();
    }
}
