package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Launcher flywheel subsystem.
 *
 * Features:
 *  - Software PIDF velocity control (derivative on filtered measurement, no kick on target change)
 *  - Battery voltage compensation of the feedforward term
 *  - Auto-feed with hysteresis: feeding pauses when speed sags, resumes once recovered
 *  - "Stopping" state: motor coasts/brakes to rest before reporting "Off"
 *
 * Status values: "Off", "Starting", "Shooting", "Stopping".
 *
 * NOTE: All PID gains and thresholds below are starting points. Tune them on the robot.
 */
public class Shooter {

    // ---------------------- Hardware & Subsystem Dependencies ----------------------
    private DcMotorEx shooter;
    private Intake intake;
    private VoltageSensor voltageSensor;

    // ---------------------- Speed Presets & Limits (ticks/sec) ----------------------
    private static final double[] LAUNCHING_POWERS = {1500, 1600, 1700, 1900};
    private static final double MAX_TARGET_VELOCITY = 2400; // set to your motor's real limit
    private int launchingIndex = 0;
    private double customTargetPower = LAUNCHING_POWERS[0];
    private boolean useCustomPower = false;

    // ---------------------- Feeding / Readiness Thresholds ----------------------
    private static final double READY_TOLERANCE = 50;      // +/- window for isReady() (display)
    private static final double PAUSE_BELOW_TARGET = 100;  // pause feeding if speed < target - this
    private static final double RESUME_BELOW_TARGET = 40;  // resume feeding when speed >= target - this
    private static final double MIN_PAUSE_SEC = 0.05;      // minimum time feeding stays paused
    private static final double STOPPED_VELOCITY = 20;     // below this the flywheel counts as stopped

    // ---------------------- Voltage Compensation ----------------------
    private static final double REFERENCE_VOLTAGE = 12.0;  // voltage kF was tuned at
    private static final double MIN_VOLTAGE = 9.0;
    private static final double MAX_VOLTAGE = 14.0;
    private static final double VOLTAGE_REFRESH_SEC = 0.25;
    private double cachedVoltage = REFERENCE_VOLTAGE;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private boolean voltageRead = false;

    // ---------------------- PIDF Coefficients ----------------------
    public double kP = 0.0015;
    public double kI = 0.0001;
    public double kD = 0.00005;
    public double kF = 0.00035;

    private static final double VELOCITY_FILTER_ALPHA = 0.4; // 1.0 = no filtering

    private double integralSum = 0;
    private double filteredVelocity = 0;
    private double lastFilteredVelocity = 0;
    private double lastPower = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private boolean pidInitialized = false;

    // ---------------------- Status ----------------------
    private String shooterStatus = "Off";
    private final ElapsedTime pauseTimer = new ElapsedTime();

    // ---------------------- Feed sequence (see startFeeding) ----------------------
    /** How long ONLY the windmill and transfer run in reverse before the first feed after start(). */
    public static final double CLEAR_REVERSE_SEC = 0.15;
    /** How long the windmill runs forward alone before the roller and transfer come on. */
    public static final double WINDMILL_SPINUP_SEC = 0.10;
    private enum FeedStage { NONE, CLEAR, WINDMILL, FEED }
    private FeedStage feedStage = FeedStage.NONE;
    private final ElapsedTime feedTimer = new ElapsedTime();
    private boolean clearedSinceStart = false;

    // =====================================================================
    // Initialization
    // =====================================================================

    /**
     * Initializes flywheel motor and voltage sensor from the hardware map.
     */
    public void init(HardwareMap hwMap, Intake intake) {
        BulkReads.enable(hwMap);
        this.intake = intake;

        shooter = hwMap.get(DcMotorEx.class, "shooter"); // throws if misconfigured (intended)
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = null;
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            voltageSensor = sensor;
            break;
        }
        voltageRead = false;

        resetPID();
    }

    // =====================================================================
    // Public control API
    // =====================================================================

    /**
     * Spools the flywheel up to the active target. Does nothing if the target is 0.
     * Safe to call while "Stopping".
     */
    public void start() {
        if (getTargetLaunchPower() <= 0) {
            shooterStatus = "Off";
            return;
        }
        shooterStatus = "Starting";
        pauseTimer.reset();
        resetPID();
        clearedSinceStart = false;   // the next time the flywheel reaches speed, clear in reverse first
        feedStage = FeedStage.NONE;
    }

    /**
     * Begins shutdown: intake stops immediately; the flywheel
     * is cut to 0 power (brake mode) and status becomes "Off" once it has slowed down.
     */
    public void stop() {
        if (!"Off".equals(shooterStatus)) {
            shooterStatus = "Stopping";
        }
        stopFeeding();
        if (shooter != null) {
            shooter.setPower(0);
        }
        lastPower = 0;
        resetPID();
    }

    public void increaseTargetSpeed(double delta) {
        customTargetPower = Math.min(MAX_TARGET_VELOCITY, getTargetLaunchPower() + delta);
        useCustomPower = true;
        resetPID();
    }

    public void decreaseTargetSpeed(double delta) {
        customTargetPower = Math.max(0, getTargetLaunchPower() - delta);
        useCustomPower = true;
        resetPID();
    }

    /**
     * Cycles preset target velocity (1500 -> 1600 -> 1700 -> 1900 ticks/sec).
     */
    public void switchSpeed() {
        launchingIndex = (launchingIndex + 1) % LAUNCHING_POWERS.length;
        customTargetPower = LAUNCHING_POWERS[launchingIndex];
        useCustomPower = false;
        resetPID();
    }

    // =====================================================================
    // Getters
    // =====================================================================

    public double getCurrentSpeed() {
        return shooter != null ? shooter.getVelocity() : 0;
    }

    public double getTargetLaunchPower() {
        return useCustomPower ? customTargetPower : LAUNCHING_POWERS[launchingIndex];
    }

    public String getShooterStatus() {
        return shooterStatus;
    }

    public int getLaunchingIndex() {
        return launchingIndex;
    }

    public double getLastPower() {
        return lastPower;
    }

    /**
     * @return Battery voltage (cached, clamped to a sane range). Falls back to the
     *         reference voltage if no sensor is available.
     */
    public double getBatteryVoltage() {
        if (voltageSensor == null) {
            return REFERENCE_VOLTAGE;
        }
        if (!voltageRead || voltageTimer.seconds() >= VOLTAGE_REFRESH_SEC) {
            double v = voltageSensor.getVoltage();
            if (!Double.isNaN(v) && v > 0) {
                cachedVoltage = Math.max(MIN_VOLTAGE, Math.min(MAX_VOLTAGE, v));
            }
            voltageRead = true;
            voltageTimer.reset();
        }
        return cachedVoltage;
    }

    /**
     * Two-sided check: speed within +/- READY_TOLERANCE of target. Intended for
     * telemetry/driver display; feeding uses the one-sided hysteresis instead.
     */
    public boolean isReady() {
        double target = getTargetLaunchPower();
        return target > 0 && Math.abs(getCurrentSpeed() - target) <= READY_TOLERANCE;
    }

    /**
     * Adds tuning telemetry. Caller is responsible for telemetry.update().
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter status", shooterStatus);
        telemetry.addData("Target (t/s)", "%.0f", getTargetLaunchPower());
        telemetry.addData("Velocity (t/s)", "%.0f", getCurrentSpeed());
        telemetry.addData("Power", "%.3f", lastPower);
        telemetry.addData("Battery (V)", "%.2f", getBatteryVoltage());
        telemetry.addData("Ready (+/-50)", isReady());
    }

    // =====================================================================
    // Main update loop - call every loop iteration
    // =====================================================================

    /** Continue the existing velocity PIDF without feeding. Call start() before using this.
     * Used by delayed autonomous: spin up while all three Intake outputs stay stopped.
     * Normal update() and all existing TeleOp behavior are unchanged. */
    public void updateSpinupOnly() {
        if (intake != null) intake.stop();
        if (intake != null) intake.update();
        if ("Starting".equals(shooterStatus) || "Shooting".equals(shooterStatus)) {
            if (getTargetLaunchPower() <= 0) stop();
            else {
                updatePIDF();
                shooterStatus = "Starting";
            }
        } else {
            update(); // Off/Stopping only; neither state permits feeding.
        }
    }

    public void update() {
        if (intake != null) intake.update();
        switch (shooterStatus) {
            case "Off":
                if (shooter != null) {
                    shooter.setPower(0);
                }
                lastPower = 0;
                break;

            case "Stopping":
                if (shooter != null) {
                    shooter.setPower(0);
                }
                lastPower = 0;
                if (getCurrentSpeed() < STOPPED_VELOCITY) {
                    shooterStatus = "Off";
                }
                break;

            default: // "Starting" or "Shooting"
                if (getTargetLaunchPower() <= 0) {
                    stop(); // target dropped to 0: wind down
                    break;
                }
                updatePIDF();
                handleAutoShooting();
                break;
        }
    }

    // =====================================================================
    // Internals
    // =====================================================================

    private void resetPID() {
        integralSum = 0;
        pidTimer.reset();
        pidInitialized = false;
    }

    /**
     * PIDF with voltage-scaled feedforward. The derivative acts on the low-pass filtered
     * measurement (not the error), so changing the target does not cause a derivative spike.
     */
    private void updatePIDF() {
        double target = getTargetLaunchPower();
        double rawVelocity = getCurrentSpeed();

        double measuredDt = pidTimer.seconds();
        pidTimer.reset();

        double dt;
        if (!pidInitialized || measuredDt <= 0.00001 || measuredDt > 0.2) {
            filteredVelocity = rawVelocity;
            lastFilteredVelocity = rawVelocity;
            pidInitialized = true;
            dt = 0.01;
        } else {
            dt = measuredDt;
            filteredVelocity += VELOCITY_FILTER_ALPHA * (rawVelocity - filteredVelocity);
        }

        double error = target - rawVelocity;

        // d(error)/dt == -d(velocity)/dt for a constant target
        double derivative = -(filteredVelocity - lastFilteredVelocity) / dt;
        lastFilteredVelocity = filteredVelocity;

        // Anti-windup: integrate only near target
        if (Math.abs(error) < 100) {
            integralSum += error * dt;
            integralSum = Math.max(-2500, Math.min(2500, integralSum));
        } else {
            integralSum = 0;
        }

        double voltageScale = REFERENCE_VOLTAGE / getBatteryVoltage();
        double ff = kF * target * voltageScale;

        double calculatedPower = ff + (kP * error) + (kI * integralSum) + (kD * derivative);
        lastPower = Math.max(0.0, Math.min(1.0, calculatedPower));

        if (shooter != null) {
            shooter.setPower(lastPower);
        }
    }

    /**
     * Feeding with hysteresis:
     *  - "Starting": feed begins once speed >= target - RESUME_BELOW_TARGET and the minimum pause elapsed.
     *  - "Shooting": feed pauses if speed < target - PAUSE_BELOW_TARGET (overshoot never pauses feeding).
     */
    private void handleAutoShooting() {
        double target = getTargetLaunchPower();
        double speed = getCurrentSpeed();

        switch (shooterStatus) {
            case "Shooting":
                if (speed < target - PAUSE_BELOW_TARGET) {
                    stopFeeding();
                    shooterStatus = "Starting";
                    pauseTimer.reset();
                } else {
                    startFeeding();
                }
                break;
            case "Starting":
                if (pauseTimer.seconds() >= MIN_PAUSE_SEC && speed >= target - RESUME_BELOW_TARGET) {
                    shooterStatus = "Shooting";
                    feedStage = FeedStage.NONE;   // begin the feed sequence fresh
                    startFeeding();
                }
                break;
        }
    }

    /**
     * Feed sequence, called every loop while "Shooting":
     *  1. CLEAR   - first time the flywheel reaches speed after start(): ONLY the windmill and transfer run in
     *               reverse for {@link #CLEAR_REVERSE_SEC} (roller off) to unload them.
     *  2. WINDMILL - the windmill alone turns forward at its feed speed for {@link #WINDMILL_SPINUP_SEC}.
     *  3. FEED    - only now the roller and transfer come on, and feeding continues.
     * After a speed-dip pause the reverse clear is skipped (steps 2 and 3 only); a new start() re-arms it.
     */
    private void startFeeding() {
        if (intake == null) return;
        intake.setWindmillFeedFromFlywheel(getTargetLaunchPower());
        switch (feedStage) {
            case NONE:
                feedTimer.reset();
                if (!clearedSinceStart) {
                    clearedSinceStart = true;
                    feedStage = FeedStage.CLEAR;
                    intake.clearReverse();
                } else {
                    feedStage = FeedStage.WINDMILL;
                    intake.startWindmillOnly();
                }
                break;
            case CLEAR:
                if (feedTimer.seconds() >= CLEAR_REVERSE_SEC) {
                    feedStage = FeedStage.WINDMILL;
                    feedTimer.reset();
                    intake.startWindmillOnly();
                }
                break;
            case WINDMILL:
                if (feedTimer.seconds() >= WINDMILL_SPINUP_SEC) {
                    feedStage = FeedStage.FEED;
                    intake.start();
                    intake.startWindmill();
                }
                break;
            case FEED:
                intake.start();
                intake.startWindmill();
                break;
        }
    }

    /**
     * Stops feeding when the flywheel dips or the shooter stops. Mid-sequence (clearing or windmill-only) everything
     * stops; during normal feeding only the windmill stops, as before.
     */
    private void stopFeeding() {
        if (intake != null) {
            if (feedStage == FeedStage.CLEAR || feedStage == FeedStage.WINDMILL) intake.stop();
            else intake.stopWindmill();
        }
        feedStage = FeedStage.NONE;
    }

    /** @return the current step of the feed sequence, for telemetry. */
    public String getFeedStage() {
        return feedStage.name();
    }
}
