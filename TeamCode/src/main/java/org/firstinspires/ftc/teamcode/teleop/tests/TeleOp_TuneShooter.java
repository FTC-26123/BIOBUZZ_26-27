package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Tunes the flywheel PIDF in {@link Shooter} with live-adjustable gains. Production code is never edited: copy the
 * printed values into {@code Shooter.java} when you are happy.
 *
 * <p>The windmill and transfer stay off (spin-up only) unless you run the shot test.
 *
 * <p>A = start/stop flywheel, dpad up/down = target speed +/-50, dpad left/right = change the selected gain,
 * bumpers = pick the gain (kF, kP, kI, kD), right trigger = 10x bigger gain steps.
 * <br>X = AUTO tune: calibrates kF, then raises kP until the speed starts to wobble and backs off.
 * <br>Y = SHOT test: feeds for about a second (put balls in!) and reports the speed dip and recovery time.
 * <br>B = abort/stop.
 */
@TeleOp(name = "Tune Shooter PIDF", group = "Tuning")
public class TeleOp_TuneShooter extends OpMode {

    private static final String[] GAIN_NAMES = {"kF", "kP", "kI", "kD"};
    private static final double[] GAIN_STEPS = {0.00001, 0.0001, 0.00001, 0.00001};

    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private int gainIndex = 1;
    private boolean running = false;

    // speed statistics (last ~1 second)
    private final ElapsedTime windowTimer = new ElapsedTime();
    private double sum, sumSq;
    private int count;
    private double windowMean, windowStdev;

    private enum Mode { MANUAL, AUTO, SHOT }
    private Mode mode = Mode.MANUAL;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Auto-tune
    private int autoStage;           // 0..2 = kF calibration passes, 3.. = kP sweep
    private double baselineStdev = -1;
    private double sweepP;
    private double origP, origI, origD;
    private String autoMessage = "";
    private static final double SETTLE_SECONDS = 2.5, MEASURE_SECONDS = 1.0;
    private static final double START_P = 0.0003, P_GROWTH = 1.4, MAX_P = 0.02;
    private static final double I_RATIO = 1.0 / 15.0, D_RATIO = 1.0 / 30.0;

    // Shot test
    private double shotMin, shotStartSpeed;
    private double shotRecovery = Double.NaN, shotDip = Double.NaN;
    private static final double SHOT_SECONDS = 1.0, RECOVER_WITHIN = 40;

    // Guided-tuning progress
    private static final double[] PRESETS = {1500, 1600, 1700, 1900};
    private static final double GOOD_DIP = 100, GOOD_RECOVERY = 0.30;
    private boolean autoDone = false, shotDone = false, lastShotGood = false;
    private final boolean[] presetOk = new boolean[PRESETS.length];

    private boolean prevA, prevB, prevX, prevY, prevUp, prevDown, prevLeft, prevRight, prevLb, prevRb;

    @Override
    public void init() {
        intake.init(hardwareMap);
        shooter.init(hardwareMap, intake);
        telemetry.addLine("Ready. A = start flywheel, X = auto tune, Y = shot test.");
    }

    @Override
    public void loop() {
        handleButtons();
        if (running) {
            if (mode == Mode.SHOT) shooter.update(); else shooter.updateSpinupOnly();
        } else {
            shooter.update();
        }
        sampleSpeed();
        if (mode == Mode.AUTO) runAuto();
        if (mode == Mode.SHOT) runShotTest();
        showTelemetry();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
    }

    // ------------------------------------------------------------------ controls

    private void handleButtons() {
        boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down, left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        boolean lb = gamepad1.left_bumper, rb = gamepad1.right_bumper;

        if (a && !prevA) {
            running = !running;
            mode = Mode.MANUAL;
            if (running) shooter.start(); else shooter.stop();
        }
        if (b && !prevB) {
            if (mode == Mode.AUTO) restoreOriginalsIfUnfinished();
            mode = Mode.MANUAL;
            running = false;
            shooter.stop();
            intake.stop();
        }
        if (x && !prevX && running && mode == Mode.MANUAL) startAuto();
        if (y && !prevY && running && mode == Mode.MANUAL) startShotTest();
        if (mode == Mode.MANUAL) {
            if (up && !prevUp) shooter.increaseTargetSpeed(50);
            if (down && !prevDown) shooter.decreaseTargetSpeed(50);
            if (lb && !prevLb) gainIndex = (gainIndex + GAIN_NAMES.length - 1) % GAIN_NAMES.length;
            if (rb && !prevRb) gainIndex = (gainIndex + 1) % GAIN_NAMES.length;
            double step = GAIN_STEPS[gainIndex] * (gamepad1.right_trigger > 0.5 ? 10 : 1);
            if (left && !prevLeft) setGain(gainIndex, Math.max(0, getGain(gainIndex) - step));
            if (right && !prevRight) setGain(gainIndex, getGain(gainIndex) + step);
        }
        prevA = a; prevB = b; prevX = x; prevY = y;
        prevUp = up; prevDown = down; prevLeft = left; prevRight = right; prevLb = lb; prevRb = rb;
    }

    private double getGain(int i) {
        switch (i) {
            case 0: return shooter.kF;
            case 1: return shooter.kP;
            case 2: return shooter.kI;
            default: return shooter.kD;
        }
    }

    private void setGain(int i, double v) {
        // Any gain change invalidates earlier shot results.
        shotDone = false; lastShotGood = false; shotDip = Double.NaN; shotRecovery = Double.NaN;
        java.util.Arrays.fill(presetOk, false);
        switch (i) {
            case 0: shooter.kF = v; break;
            case 1: shooter.kP = v; break;
            case 2: shooter.kI = v; break;
            default: shooter.kD = v; break;
        }
    }

    // ------------------------------------------------------------------ measurement

    private void resetWindow() {
        sum = 0; sumSq = 0; count = 0;
        windowTimer.reset();
    }

    private void sampleSpeed() {
        double v = shooter.getCurrentSpeed();
        sum += v; sumSq += v * v; count++;
        if (windowTimer.seconds() >= MEASURE_SECONDS && count > 3) {
            windowMean = sum / count;
            windowStdev = Math.sqrt(Math.max(0, sumSq / count - windowMean * windowMean));
            resetWindow();
        }
    }

    // ------------------------------------------------------------------ auto tune

    private void startAuto() {
        origP = shooter.kP; origI = shooter.kI; origD = shooter.kD;
        shooter.kP = 0; shooter.kI = 0; shooter.kD = 0;   // pure feedforward for the kF calibration
        autoStage = 0;
        baselineStdev = -1;
        sweepP = START_P;
        autoMessage = "Calibrating kF (pure feedforward)...";
        mode = Mode.AUTO;
        stateTimer.reset();
        resetWindow();
        windowMean = 0; windowStdev = 0;
    }

    private void restoreOriginalsIfUnfinished() {
        shooter.kP = origP; shooter.kI = origI; shooter.kD = origD;
        autoMessage = "Aborted: kP/kI/kD restored (kF kept if calibrated).";
    }

    private void runAuto() {
        // Wait until a fresh measurement window that began after settling is ready.
        if (stateTimer.seconds() < SETTLE_SECONDS + MEASURE_SECONDS + 0.2) return;
        double target = shooter.getTargetLaunchPower();
        if (target <= 0) { restoreOriginalsIfUnfinished(); mode = Mode.MANUAL; return; }

        if (autoStage < 3) {
            if (windowMean > 50) {
                double ratio = Math.max(0.5, Math.min(2.0, target / windowMean));
                shooter.kF *= ratio;
            }
            autoStage++;
            if (autoStage == 3) {
                shooter.kP = sweepP; shooter.kI = 0; shooter.kD = 0;
                autoMessage = "kF calibrated. Raising kP until the speed wobbles...";
            }
            nextMeasurement();
            return;
        }

        // kP sweep: first step sets the noise baseline.
        if (baselineStdev < 0) {
            baselineStdev = Math.max(8, windowStdev);
        } else {
            boolean wobbling = windowStdev > Math.max(30, 3 * baselineStdev)
                    || Math.abs(windowMean - target) > 150;
            if (wobbling || sweepP >= MAX_P) {
                double chosen = (wobbling ? sweepP * 0.5 : MAX_P * 0.5);
                applyFinal(chosen, wobbling ? sweepP : -1);
                return;
            }
        }
        sweepP *= P_GROWTH;
        shooter.kP = sweepP;
        nextMeasurement();
    }

    private void nextMeasurement() {
        stateTimer.reset();
        resetWindow();
        windowMean = 0; windowStdev = 0;
    }

    private void applyFinal(double p, double wobbleP) {
        shooter.kP = p;
        shooter.kI = p * I_RATIO;
        shooter.kD = p * D_RATIO;
        autoMessage = wobbleP > 0
                ? String.format("Wobble started at kP=%.5f. Using half of that. Now run the shot test (Y).", wobbleP)
                : "No wobble up to the max kP. Using half of max. Now run the shot test (Y).";
        mode = Mode.MANUAL;
        autoDone = true;
        shotDone = false; lastShotGood = false; shotDip = Double.NaN; shotRecovery = Double.NaN;
        java.util.Arrays.fill(presetOk, false);
    }

    // ------------------------------------------------------------------ shot test

    private void startShotTest() {
        mode = Mode.SHOT;
        stateTimer.reset();
        shotMin = Double.MAX_VALUE;
        shotStartSpeed = shooter.getCurrentSpeed();
        shotRecovery = Double.NaN;
        shotDip = Double.NaN;
    }

    private void runShotTest() {
        double t = stateTimer.seconds();
        double v = shooter.getCurrentSpeed();
        double target = shooter.getTargetLaunchPower();
        if (t > 0.15) shotMin = Math.min(shotMin, v);
        if (Double.isNaN(shotRecovery) && shotMin < target - RECOVER_WITHIN * 2 && v >= target - RECOVER_WITHIN) {
            shotRecovery = t;
        }
        if (t >= SHOT_SECONDS) {
            shotDip = shotMin == Double.MAX_VALUE ? 0 : target - shotMin;
            shotDone = true;
            lastShotGood = shotDip <= GOOD_DIP && !Double.isNaN(shotRecovery) && shotRecovery <= GOOD_RECOVERY
                    || shotDip < RECOVER_WITHIN * 2;   // a tiny dip needs no recovery
            if (lastShotGood) {
                for (int i = 0; i < PRESETS.length; i++) {
                    if (Math.abs(target - PRESETS[i]) <= 25) presetOk[i] = true;
                }
            }
            mode = Mode.MANUAL;
            shooter.start();   // return to spin-up only; the next loop stops the intake
        }
    }

    // ------------------------------------------------------------------ telemetry

    /** Prints exact next-step instructions based on where the tuning is. */
    private void showGuide() {
        telemetry.addLine("=========== WHAT TO DO NEXT ===========");
        double target = shooter.getTargetLaunchPower();
        boolean wobbling = windowStdev > Math.max(30, 3 * Math.max(8, baselineStdev));
        if (!running) {
            telemetry.addLine("STEP 1: Press A on gamepad 1. The flywheel starts. No balls needed yet.");
            telemetry.addLine("Then wait about 5 seconds for 'Speed' to settle near 'Target'.");
        } else if (mode == Mode.AUTO) {
            telemetry.addLine("STEP 2: AUTO-TUNE IS RUNNING (about 20-40 s). Do not press anything.");
            telemetry.addLine("Press B only if you need to abort.");
        } else if (mode == Mode.SHOT) {
            telemetry.addLine("SHOT TEST RUNNING (1 s). Do not press anything. Results appear below.");
        } else if (!autoDone) {
            telemetry.addLine("STEP 2: Set Target to the speed you shoot at most (dpad UP/DOWN = +/-50).");
            telemetry.addLine("Wait until 'Avg' is within 30 of Target, then press X for auto-tune.");
        } else if (!shotDone) {
            telemetry.addLine("STEP 3: Auto-tune finished. Put 3 balls in the robot.");
            telemetry.addLine("Press Y. The robot feeds for 1 s. Then read Dip and Recovery below.");
            telemetry.addLine("Goal: Dip under 100 and Recovery under 0.30 s.");
        } else if (wobbling) {
            telemetry.addLine("STEP 4: SPEED IS WOBBLING (wobble number is high).");
            telemetry.addLine("1) Press RB or LB until the '>' arrow is on kP.");
            telemetry.addLine("2) Press dpad LEFT 2 times.  3) Wait 5 s.  4) Reload balls, press Y.");
        } else if (!lastShotGood) {
            telemetry.addLine("STEP 4: SPEED DROPS TOO FAR OR RECOVERS TOO SLOWLY AFTER A SHOT.");
            telemetry.addLine("1) Press RB or LB until the '>' arrow is on kP.");
            telemetry.addLine("2) Press dpad RIGHT 2 times (each press = +0.0001; hold right trigger = 10x).");
            telemetry.addLine("3) Wait 5 s.  4) Reload 3 balls.  5) Press Y again.");
            telemetry.addLine("If it then wobbles, press dpad LEFT 1 time. Repeat until it passes.");
        } else {
            StringBuilder s = new StringBuilder("Passed at:");
            double next = -1;
            for (int i = 0; i < PRESETS.length; i++) {
                s.append(String.format(" %.0f[%s]", PRESETS[i], presetOk[i] ? "x" : " "));
                if (!presetOk[i] && next < 0) next = PRESETS[i];
            }
            telemetry.addLine("STEP 5: THIS SHOT PASSED. " + s);
            if (next > 0) {
                int presses = (int) Math.round((next - target) / 50.0);
                telemetry.addLine(String.format("Next: set Target to %.0f (press dpad %s %d time%s).", next,
                        presses >= 0 ? "UP" : "DOWN", Math.abs(presses), Math.abs(presses) == 1 ? "" : "s"));
                telemetry.addLine("Wait 5 s for the speed to settle, reload 3 balls, press Y.");
                telemetry.addLine("If a speed fails, follow the fix steps shown for it.");
            } else {
                telemetry.addLine("STEP 6: ALL SPEEDS PASSED. You are done.");
                telemetry.addLine("Copy the kP/kI/kD/kF line below into Shooter.java (public double kP... fields).");
                telemetry.addLine("Then press STOP on the driver station.");
            }
        }
        telemetry.addLine("=======================================");
    }

    private void showTelemetry() {
        showGuide();
        telemetry.addData("Mode", running ? mode : "STOPPED");
        telemetry.addData("Target (t/s)", "%.0f", shooter.getTargetLaunchPower());
        telemetry.addData("Speed (t/s)", "%.0f", shooter.getCurrentSpeed());
        telemetry.addData("Avg / wobble (last 1 s)", "%.0f / %.1f", windowMean, windowStdev);
        telemetry.addData("Power", "%.3f", shooter.getLastPower());
        telemetry.addData("Battery (V)", "%.2f", shooter.getBatteryVoltage());
        for (int i = 0; i < GAIN_NAMES.length; i++) {
            telemetry.addData((i == gainIndex ? "> " : "  ") + GAIN_NAMES[i], "%.6f", getGain(i));
        }
        if (!autoMessage.isEmpty()) telemetry.addData("Auto", autoMessage);
        if (mode == Mode.AUTO) telemetry.addData("Auto step", autoStage < 3 ? "kF pass " + (autoStage + 1) + "/3" : String.format("kP %.5f", sweepP));
        if (!Double.isNaN(shotDip)) {
            telemetry.addData("Last shot: dip (t/s)", "%.0f", shotDip);
            telemetry.addData("Last shot: recovery (s)", Double.isNaN(shotRecovery) ? "not within window" : String.format("%.2f", shotRecovery));
        }
        telemetry.addLine("--- Copy into Shooter.java if you like it ---");
        telemetry.addLine(String.format("kP = %.6f;  kI = %.6f;  kD = %.6f;  kF = %.6f;",
                shooter.kP, shooter.kI, shooter.kD, shooter.kF));
        telemetry.addLine("A start/stop | X auto | Y shot test | B abort | dpad up/down speed | dpad L/R gain | bumpers pick gain");
        telemetry.update();
    }
}
