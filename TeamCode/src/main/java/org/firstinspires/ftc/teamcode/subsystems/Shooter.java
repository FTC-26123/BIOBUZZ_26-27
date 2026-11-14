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
 *  - One ball at a time: feeding only happens while the flywheel is inside its speed window. The moment a shot
 *    pulls the speed down, the gate closes (windmill braked, transfer stopped) and reopens as soon as the speed
 *    is back, so every ball leaves at the target speed with no extra waiting
 *  - Flywheel-only mode ("Spinning") so TeleOp can keep the wheel at speed between volleys
 *  - Speed presets with a per-preset trim the drivers can change during a match
 *
 * Status values: "Off", "Starting", "Shooting", "Spinning", "Stopping".
 *   Starting = fire requested, waiting for the flywheel (gate closed)
 *   Shooting = fire requested and flywheel in its window (feeding)
 *   Spinning = flywheel held at speed, not feeding (TeleOp only)
 *
 * NOTE: All PID gains and thresholds below are starting points. Tune them on the robot.
 */
public class Shooter {

    // ---------------------- Hardware & Subsystem Dependencies ----------------------
    private DcMotorEx shooter;
    private Intake intake;
    private VoltageSensor voltageSensor;

    // ---------------------- Speed Presets & Limits (ticks/sec) ----------------------
    /** Preset speeds. Editable on the robot (Robot Settings OpMode), so the contents can change at run time. */
    public static final double[] LAUNCHING_POWERS = {1500, 1600, 1700, 1900};
    private static final double MAX_TARGET_VELOCITY = 2400; // set to your motor's real limit
    /** One press of a driver trim button changes the current preset by this much (ticks/sec). */
    public static double TRIM_STEP = 25;
    /** Driver trim added to each preset; kept per preset so a correction made at one spot is still there later. */
    private final double[] presetTrim = new double[LAUNCHING_POWERS.length];
    private int launchingIndex = 0;
    /**
     * AUTO speed (set by AutoShot from the camera distance). While active it replaces the preset; the driver's
     * trim buttons then move {@link #autoTrim}, one offset that applies at every distance.
     */
    private boolean autoActive = false;
    private double autoBase = 0, autoTrim = 0;
    /** false = hold fire even at speed (AutoShot: not pointed at the basket yet). Always true unless set. */
    private boolean feedPermit = true;
    /** Balls counted as shot: the flywheel speed dropped while the feed was open (see getShotCount). */
    private int shotCount = 0;
    /** Slow average of the flywheel speed while feeding: the level a shot's drop is measured from. */
    private double feedReference = 0;
    private boolean feedReferenceValid = false;

    // ---------------------- Feeding / Readiness Thresholds ----------------------
    private static final double READY_TOLERANCE = 50;      // +/- window for isReady() (display)
    /**
     * Gate always closes (feeding pauses) when speed < target - this. A smaller sudden drop is caught earlier
     * by {@link #SHOT_DROP}. (The older, now hidden autonomous programs count shots with PreloadShotCounter,
     * which expects to see the drop before the gate closes; BioBuzzAuto uses getShotCount() instead.)
     */
    public static double PAUSE_BELOW_TARGET = 100;
    public static double RESUME_BELOW_TARGET = 40;  // gate may reopen when speed >= target - this ...
    public static double RESUME_ABOVE_TARGET = 80;  // ... and speed <= target + this (no over-speed shots)
    public static double PAUSE_ABOVE_TARGET = 150;  // gate closes when speed > target + this (target lowered)
    private static final double MIN_PAUSE_SEC = 0.05;      // minimum time feeding stays paused
    /**
     * Extra wait between balls: after a ball goes through, the feed reopens only once the flywheel has been back
     * at speed for this long (seconds). 0 = reopen the moment the speed is back. Robot Settings > Shooter gate.
     */
    public static double BETWEEN_BALLS_SEC = 0.1;
    private double betweenBallsOverride = Double.NaN;
    private final ElapsedTime inWindowTimer = new ElapsedTime();
    private boolean wasInWindow = false, afterShot = false;
    /**
     * A ball going through shows as a sudden drop in flywheel speed. While feeding, a drop of at least this many
     * ticks/sec below the steady feeding speed counts as one shot and closes the gate until the speed is back,
     * even when the speed is still above target - PAUSE_BELOW_TARGET. Lower it if balls are not being counted
     * (the autonomous then waits for its "no more balls" timeout); raise it if the gate closes with no ball
     * (speed ripple). The hub reports speed in steps of about 20, so keep it above 40.
     */
    public static double SHOT_DROP = 45;
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
    // Code defaults only. The values used are the ones saved on the robot (Robot Settings OpMode or the
    // "Shooter PIDF Manual" tuning OpMode), so changing them never needs a rebuild.
    // Flywheel control as in "PID Control Full Course: Flywheels" (B. Pratt): power = kS + kV x target + kP x error.
    // kS = power that just starts the wheel turning, kV (field name kF) = power per tick/sec, kP = correction.
    // Integral and derivative are not used on a flywheel (leave kI and kD at 0).
    public static double kS = 0.18;
    public static double kP = 0.0045;
    public static double kI = 0.0;
    public static double kD = 0.0;
    /** kV: feedforward power per tick/sec of target (called kF in older code). */
    public static double kF = 0.000480;

    /** Tuning only: NORMAL = full controller, KS_ONLY = power is just kS, FEEDFORWARD_ONLY = kS + kV, no kP. */
    public enum TuneMode { NORMAL, KS_ONLY, FEEDFORWARD_ONLY }
    private TuneMode tuneMode = TuneMode.NORMAL;

    /** Tuning OpModes only. Every other OpMode leaves this at NORMAL. */
    public void setTuneMode(TuneMode mode) {
        tuneMode = mode == null ? TuneMode.NORMAL : mode;
        integralSum = 0;
    }

    private static final double VELOCITY_FILTER_ALPHA = 0.4; // 1.0 = no filtering
    private static final double INTEGRAL_ZONE = 100;         // integrate only within this error
    private static final double INTEGRAL_LIMIT = 2500;

    private double integralSum = 0;
    private double filteredVelocity = 0;
    private double lastFilteredVelocity = 0;
    private double lastPower = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private boolean pidInitialized = false;
    /** Flywheel speed sampled once per update(); everything inside one loop uses this same reading. */
    private double velocity = 0;

    // ---------------------- Status ----------------------
    private static final String OFF = "Off", STARTING = "Starting", SHOOTING = "Shooting",
            SPINNING = "Spinning", STOPPING = "Stopping";
    private String shooterStatus = OFF;
    private final ElapsedTime pauseTimer = new ElapsedTime();
    /** true = feed whenever the flywheel is in its window; false = flywheel only. */
    private boolean feedRequested = false;

    // ---------------------- Feed sequence (see advanceFeed) ----------------------
    /**
     * How long ONLY the windmill and transfer run in reverse before the first feed of a volley, to take the
     * balls' weight off the windmill. It runs while the flywheel is still spinning up when possible, and only
     * when the intake has run (or the shooter was started) since the last volley.
     */
    public static double CLEAR_REVERSE_SEC = 0.0;
    /** First feed of a volley: how long the windmill runs forward alone before the roller and transfer come on. */
    public static double WINDMILL_SPINUP_SEC = 0.0;
    /**
     * Same head start for the windmill when feeding resumes BETWEEN shots. 0 = none: the windmill goes straight
     * to feed power and the transfer soft-starts behind it. Raise it a little (0.03) only if the windmill stalls
     * on the second and later balls.
     */
    public static double WINDMILL_RESUME_SEC = 0.0;

    /** NONE = shooter is not driving the intake; HOLD = gate closed, waiting for flywheel speed. */
    private enum FeedStage { NONE, CLEAR, HOLD, WINDMILL, FEED }
    private FeedStage feedStage = FeedStage.NONE;
    private final ElapsedTime feedTimer = new ElapsedTime();
    private boolean needsClear = true;       // reverse-clear before the next feed
    private boolean firstFeed = true;        // next feed is the first of a volley (uses WINDMILL_SPINUP_SEC)
    private boolean driverIntakeWasOn = false; // roller state to restore when feeding ends
    private double windmillLead = 0;
    private double gateTarget = 0;             // target the gate was last judged against

    // =====================================================================
    // Initialization
    // =====================================================================

    /**
     * Initializes flywheel motor and voltage sensor from the hardware map.
     */
    public void init(HardwareMap hwMap, Intake intake) {
        RobotSettings.ensureLoaded();
        BulkReads.enable(hwMap);
        this.intake = intake;

        shooter = hwMap.get(DcMotorEx.class, "shooter"); // throws if misconfigured (intended)
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // FLOAT: a flywheel should coast, not be braked. Braking wastes the stored speed and makes the next
        // spin-up start from zero.
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        voltageSensor = null;
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            voltageSensor = sensor;
            break;
        }
        voltageRead = false;

        shooterStatus = OFF;
        feedRequested = false;
        feedStage = FeedStage.NONE;
        needsClear = true;
        firstFeed = true;
        autoActive = false;
        feedPermit = true;
        resetPID();
    }

    // =====================================================================
    // Public control API
    // =====================================================================

    /**
     * FIRE: spools the flywheel up to the active target and feeds whenever it is inside its speed window.
     * Safe to call every loop (holding a trigger): it does nothing if firing is already requested.
     * Does nothing if the target is 0. Safe to call while "Stopping".
     */
    public void start() {
        if (getTargetLaunchPower() <= 0) {
            shooterStatus = OFF;
            return;
        }
        if (feedRequested && (STARTING.equals(shooterStatus) || SHOOTING.equals(shooterStatus))) return;
        feedRequested = true;
        afterShot = false;   // the first ball of a volley does not wait
        if (SPINNING.equals(shooterStatus) || STARTING.equals(shooterStatus) || SHOOTING.equals(shooterStatus)) {
            // Flywheel already running: keep its PID state, the gate opens as soon as the speed is in the window.
            shooterStatus = STARTING;
            return;
        }
        shooterStatus = STARTING;
        pauseTimer.reset();
        resetPID();
        needsClear = true;    // unload the windmill once before this volley
        firstFeed = true;
        feedStage = FeedStage.NONE;
    }

    /**
     * FLYWHEEL ONLY: brings the flywheel to the target speed and keeps it there without feeding. Use it to be
     * ready before reaching the shooting spot. If the shooter is firing, this stops the feeding and keeps the
     * flywheel running.
     */
    public void spinUp() {
        if (getTargetLaunchPower() <= 0) return;
        if (OFF.equals(shooterStatus) || STOPPING.equals(shooterStatus)) {
            pauseTimer.reset();
            resetPID();
            needsClear = true;
            firstFeed = true;
            feedRequested = false;
            feedStage = FeedStage.NONE;
            shooterStatus = SPINNING;
        } else {
            holdFire();
        }
    }

    /**
     * Stops feeding but keeps the flywheel at speed ("Spinning"), ready for the next volley. Does nothing unless
     * the shooter is firing. The intake goes back to what the driver had before the volley (running or stopped).
     */
    public void holdFire() {
        if (!feedRequested) return;
        feedRequested = false;
        if (STARTING.equals(shooterStatus) || SHOOTING.equals(shooterStatus)) {
            if (feedStage == FeedStage.CLEAR) needsClear = true;   // the clear was cut short; redo it next time
            releaseIntake();
            firstFeed = true;
            shooterStatus = SPINNING;
        }
    }

    /**
     * Begins shutdown: feeding stops immediately; the flywheel is cut to 0 power and coasts, and status becomes
     * "Off" once it has slowed down.
     */
    public void stop() {
        if (!OFF.equals(shooterStatus)) {
            shooterStatus = STOPPING;
        }
        feedRequested = false;
        releaseIntake();
        if (shooter != null) {
            shooter.setPower(0);
        }
        lastPower = 0;
        resetPID();
    }

    /**
     * Raises the trim (ticks/sec). In manual mode it changes the current preset and stays with that preset;
     * in AUTO speed mode it changes the one auto trim that is added at every distance.
     */
    public void increaseTargetSpeed(double delta) {
        double wanted = Math.max(0, Math.min(MAX_TARGET_VELOCITY, getTargetLaunchPower() + delta));
        if (autoActive) autoTrim = wanted - autoBase;
        else presetTrim[launchingIndex] = wanted - LAUNCHING_POWERS[launchingIndex];
    }

    /** Lowers the trim of the current preset (ticks/sec). The trim stays with that preset. */
    public void decreaseTargetSpeed(double delta) {
        increaseTargetSpeed(-delta);
    }

    /** Removes the driver trim from the current preset. */
    public void resetTrim() {
        if (autoActive) autoTrim = 0;
        else presetTrim[launchingIndex] = 0;
    }

    /**
     * AUTO speed: use this flywheel speed (from the distance table) instead of the preset. The driver's auto
     * trim is added on top. Stays active until {@link #clearAutoTarget()} or a preset button is pressed.
     */
    public void setAutoTarget(double ticksPerSec) {
        if (Double.isNaN(ticksPerSec) || ticksPerSec <= 0) return;
        autoBase = Math.min(MAX_TARGET_VELOCITY, ticksPerSec);
        autoActive = true;
    }

    /** Per-OpMode recovery wait; leaves Robot Settings and other shooter instances unchanged. */
    public void setMinimumBetweenBallsSeconds(double seconds) {
        if (!Double.isFinite(seconds) || seconds < 0) throw new IllegalArgumentException("Invalid between-ball delay");
        betweenBallsOverride = seconds;
    }

    /** Back to the manual presets. */
    public void clearAutoTarget() {
        autoActive = false;
    }

    /** @return true while the speed comes from the distance table rather than a preset. */
    public boolean isAutoTarget() {
        return autoActive;
    }

    /** @return the table speed last given to {@link #setAutoTarget} (without the driver trim). */
    public double getAutoBaseSpeed() {
        return autoBase;
    }

    /**
     * Extra condition for feeding, on top of flywheel speed. false = keep the gate closed (for example: not
     * pointed at the basket yet). The flywheel keeps running. Defaults to true; autonomous never changes it.
     */
    public void setFeedPermit(boolean permit) {
        feedPermit = permit;
    }

    /**
     * Next preset (1500 -> 1600 -> 1700 -> 1900 -> 1500 ticks/sec, each plus its own trim).
     */
    public void switchSpeed() {
        if (autoActive) { autoActive = false; return; }   // first press: leave AUTO speed, keep the current preset
        launchingIndex = (launchingIndex + 1) % LAUNCHING_POWERS.length;
    }

    /** Previous preset. */
    public void previousSpeed() {
        if (autoActive) { autoActive = false; return; }   // first press: leave AUTO speed, keep the current preset
        launchingIndex = (launchingIndex + LAUNCHING_POWERS.length - 1) % LAUNCHING_POWERS.length;
    }

    // =====================================================================
    // Getters
    // =====================================================================

    /**
     * Fresh flywheel speed straight from the motor. Autonomous shot counting relies on this being a new reading.
     * For display and anything inside the control loop prefer {@link #getLastSpeed()}, which costs no hub read.
     */
    public double getCurrentSpeed() {
        return shooter != null ? shooter.getVelocity() : 0;
    }

    /** Flywheel speed as sampled by the most recent update() (no extra hub read). */
    public double getLastSpeed() {
        return velocity;
    }

    public double getTargetLaunchPower() {
        double target = autoActive ? autoBase + autoTrim
                : LAUNCHING_POWERS[launchingIndex] + presetTrim[launchingIndex];
        return Math.max(0, Math.min(MAX_TARGET_VELOCITY, target));
    }

    public String getShooterStatus() {
        return shooterStatus;
    }

    public int getLaunchingIndex() {
        return launchingIndex;
    }

    public int getPresetCount() {
        return LAUNCHING_POWERS.length;
    }

    /** Untrimmed speed of the current preset. */
    public double getPresetSpeed() {
        return LAUNCHING_POWERS[launchingIndex];
    }

    /** Driver trim in effect (ticks/sec, may be negative): the preset's trim, or the auto trim in AUTO speed. */
    public double getPresetTrim() {
        return getTargetLaunchPower() - (autoActive ? autoBase : LAUNCHING_POWERS[launchingIndex]);
    }

    /** One line for the driver: "2/4  1600 +25 = 1625", or "AUTO  1640 +25 = 1665" in AUTO speed. */
    public String getPresetText() {
        if (autoActive) {
            return String.format(java.util.Locale.US, "AUTO  %.0f %+.0f = %.0f", autoBase, getPresetTrim(),
                    getTargetLaunchPower());
        }
        return String.format(java.util.Locale.US, "%d/%d  %.0f %+.0f = %.0f", launchingIndex + 1,
                LAUNCHING_POWERS.length, getPresetSpeed(), getPresetTrim(), getTargetLaunchPower());
    }

    public double getLastPower() {
        return lastPower;
    }

    /** @return true while firing is requested (feeding whenever the flywheel is in its window). */
    public boolean isFiring() {
        return feedRequested && (STARTING.equals(shooterStatus) || SHOOTING.equals(shooterStatus));
    }

    /** @return true while the flywheel is being driven (Starting, Shooting or Spinning). */
    public boolean isRunning() {
        return STARTING.equals(shooterStatus) || SHOOTING.equals(shooterStatus) || SPINNING.equals(shooterStatus);
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
     * Two-sided check: speed within +/- READY_TOLERANCE of target, using the speed from the last update().
     * Intended for telemetry/driver display; feeding uses the hysteresis window instead.
     */
    public boolean isReady() {
        double target = getTargetLaunchPower();
        return isRunning() && target > 0 && Math.abs(velocity - target) <= READY_TOLERANCE;
    }

    /**
     * Adds tuning telemetry. Caller is responsible for telemetry.update().
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter status", shooterStatus);
        telemetry.addData("Preset", getPresetText());
        telemetry.addData("Target (t/s)", "%.0f", getTargetLaunchPower());
        telemetry.addData("Velocity (t/s)", "%.0f", velocity);
        telemetry.addData("Power", "%.3f", lastPower);
        telemetry.addData("Battery (V)", "%.2f", getBatteryVoltage());
        telemetry.addData("Ready (+/-50)", isReady());
        telemetry.addData("Feed step", feedStage.name());
    }

    // =====================================================================
    // Main update loop - call every loop iteration
    // =====================================================================

    /** Continue the existing velocity PIDF without feeding. Call start() before using this.
     * Used by delayed autonomous: spin up while all three Intake outputs stay stopped. */
    public void updateSpinupOnly() {
        if (intake != null) intake.stop();
        if (intake != null) intake.update();
        feedStage = FeedStage.NONE;   // the intake was just stopped; the next update() starts the sequence again
        velocity = getCurrentSpeed();
        if (isRunning()) {
            if (getTargetLaunchPower() <= 0) stop();
            else {
                if (intake != null) intake.setBatteryVoltage(getBatteryVoltage());
                updatePIDF();
                shooterStatus = STARTING;
            }
        } else {
            idle();
        }
    }

    public void update() {
        if (intake != null) intake.update();
        velocity = getCurrentSpeed();   // the only flywheel speed read this loop
        if (!isRunning()) {
            idle();
            return;
        }
        if (getTargetLaunchPower() <= 0) {
            stop(); // target dropped to 0: wind down
            return;
        }
        if (intake != null) intake.setBatteryVoltage(getBatteryVoltage());
        updatePIDF();
        if (feedRequested) {
            handleAutoShooting();
        } else {
            // Flywheel only. The driver owns the intake; remember that balls were moved so the next volley
            // unloads the windmill first.
            shooterStatus = SPINNING;
            if (intake != null && intake.isRollerIn()) needsClear = true;
        }
    }

    // =====================================================================
    // Internals
    // =====================================================================

    /** "Off" and "Stopping": no power; Stopping becomes Off once the wheel has coasted down. */
    private void idle() {
        if (shooter != null) {
            shooter.setPower(0);
        }
        lastPower = 0;
        if (STOPPING.equals(shooterStatus) && velocity < STOPPED_VELOCITY) {
            shooterStatus = OFF;
        }
    }

    /** Clears the integral term (tuning: call after changing gains so old windup does not carry over). */
    public void resetIntegral() {
        integralSum = 0;
    }

    private void resetPID() {
        integralSum = 0;
        pidTimer.reset();
        pidInitialized = false;
    }

    /**
     * Plain PIDF with voltage-scaled feedforward. The derivative acts on the low-pass
     * filtered measurement (not the error), so changing the target does not cause a derivative spike.
     */
    private void updatePIDF() {
        double target = getTargetLaunchPower();
        double rawVelocity = velocity;

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

        // Anti-windup: integrate only near target. Outside the zone the integral is HELD, not cleared, so the
        // trim it has learned is still there when the speed comes back after a shot.
        if (Math.abs(error) < INTEGRAL_ZONE) {
            integralSum += error * dt;
            integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
        }

        double voltageScale = REFERENCE_VOLTAGE / getBatteryVoltage();
        double ff = (kS + kF * target) * voltageScale;
        if (tuneMode == TuneMode.KS_ONLY) {
            lastPower = Math.max(0.0, Math.min(1.0, kS * voltageScale));
            if (shooter != null) shooter.setPower(lastPower);
            return;
        }
        if (tuneMode == TuneMode.FEEDFORWARD_ONLY) {
            lastPower = Math.max(0.0, Math.min(1.0, ff));
            if (shooter != null) shooter.setPower(lastPower);
            return;
        }

        double calculatedPower = ff + (kP * error) + (kI * integralSum) + (kD * derivative);
        lastPower = Math.max(0.0, Math.min(1.0, calculatedPower));

        if (shooter != null) {
            shooter.setPower(lastPower);
        }
    }

    /**
     * Speed gate with hysteresis, then the feed sequence:
     *  - "Starting": the gate opens once the minimum pause has passed and the speed is inside
     *                [target - RESUME_BELOW_TARGET, target + RESUME_ABOVE_TARGET].
     *  - "Shooting": the gate closes when speed < target - PAUSE_BELOW_TARGET (a ball just went through, or the
     *                battery sagged), speed > target + PAUSE_ABOVE_TARGET, or the driver changed the target and the
     *                speed is not inside the new window yet.
     */
    private void handleAutoShooting() {
        double target = getTargetLaunchPower();
        double speed = velocity;

        boolean inWindow = speed >= target - RESUME_BELOW_TARGET && speed <= target + RESUME_ABOVE_TARGET;
        // A changed target (preset or trim) must be re-earned: never feed at the old speed.
        boolean targetMoved = target != gateTarget && !inWindow;
        gateTarget = target;

        if (SHOOTING.equals(shooterStatus)) {
            // A speed drop while the feed is open is a ball going through the flywheel: count it and close the
            // gate so the next ball waits for the speed to come back.
            boolean shot = false;
            if (feedStage == FeedStage.FEED) {
                if (!feedReferenceValid) {
                    feedReference = speed;
                    feedReferenceValid = true;
                }
                shot = speed <= feedReference - SHOT_DROP || speed < target - PAUSE_BELOW_TARGET;
                if (shot) shotCount++;
                else feedReference += 0.15 * (speed - feedReference);
            } else {
                feedReferenceValid = false;
            }
            if (shot || speed < target - PAUSE_BELOW_TARGET || speed > target + PAUSE_ABOVE_TARGET || targetMoved) {
                if (shot) {
                    afterShot = true;
                    // A counted load drop can stay inside the speed window. Start
                    // the next-ball wait now instead of reusing an old window timer.
                    inWindowTimer.reset();
                }
                shooterStatus = STARTING;
                pauseTimer.reset();
                feedReferenceValid = false;
            }
        } else {
            feedReferenceValid = false;
            shooterStatus = STARTING;
            if (inWindow && !wasInWindow) inWindowTimer.reset();
            // Between balls, the speed must have been back for BETWEEN_BALLS_SEC before the next ball goes.
            double recoveryWait = Double.isNaN(betweenBallsOverride) ? BETWEEN_BALLS_SEC : betweenBallsOverride;
            boolean settled = !afterShot || inWindowTimer.seconds() >= recoveryWait;
            if (pauseTimer.seconds() >= MIN_PAUSE_SEC && inWindow && settled) {
                shooterStatus = SHOOTING;
                afterShot = false;
            }
        }
        wasInWindow = inWindow;
        advanceFeed(SHOOTING.equals(shooterStatus) && feedPermit);
    }

    /**
     * Feed sequence, called every loop while firing is requested. {@code gateOpen} is true while the flywheel is
     * inside its speed window.
     *  1. CLEAR    - once per volley (only if the intake ran or the shooter was started since the last one): ONLY
     *                the windmill and transfer run in reverse for {@link #CLEAR_REVERSE_SEC} (roller off) to
     *                unload them. This starts right away, so it overlaps the flywheel spin-up.
     *  2. HOLD     - gate closed: windmill braked, transfer stopped. Nothing can reach the flywheel.
     *  3. WINDMILL - gate open: the windmill alone turns forward for a short head start
     *                ({@link #WINDMILL_SPINUP_SEC} for the first ball, {@link #WINDMILL_RESUME_SEC} after that).
     *  4. FEED     - roller and transfer on. The moment the flywheel speed drops (a ball went through) the gate
     *                closes again: back to HOLD until the speed has recovered.
     */
    private void advanceFeed(boolean gateOpen) {
        if (intake == null) return;
        intake.setWindmillFeedFromFlywheel(getTargetLaunchPower());

        if (feedStage == FeedStage.NONE) {
            // Taking the intake over from the driver: remember whether it was running so it can be restored.
            driverIntakeWasOn = intake.isRollerIn();
            if (driverIntakeWasOn) needsClear = true;
            // The reverse clear only runs if it has a time set. At 0 it is skipped completely (it used to reverse
            // the windmill and transfer for one loop anyway, pushing balls back out when firing started).
            if (needsClear && CLEAR_REVERSE_SEC > 0) {
                needsClear = false;
                firstFeed = true;
                feedStage = FeedStage.CLEAR;
                feedTimer.reset();
                intake.clearReverse();
                return;
            }
            closeGate();
        }

        switch (feedStage) {
            case CLEAR:
                if (feedTimer.seconds() >= CLEAR_REVERSE_SEC) closeGate();
                else break;
                // fall through: the gate may open in this same loop
            case HOLD:
                if (!intake.isHoldingFeed()) intake.holdFeed();   // re-assert if a driver command changed it
                if (gateOpen) openGate();
                break;
            case WINDMILL:
                if (!gateOpen) closeGate();
                else if (feedTimer.seconds() >= windmillLead) beginFeed();
                break;
            case FEED:
                if (!gateOpen) {
                    closeGate();
                } else {
                    intake.start();
                    intake.startWindmill();
                }
                break;
            default:
                break;
        }
    }

    private void closeGate() {
        feedStage = FeedStage.HOLD;
        intake.holdFeed();
    }

    private void openGate() {
        windmillLead = firstFeed ? WINDMILL_SPINUP_SEC : WINDMILL_RESUME_SEC;
        if (windmillLead <= 0) {
            // Between shots: windmill straight back to feed power, transfer soft-starts behind it.
            intake.resumeWindmill();
            intake.start();
            feedStage = FeedStage.FEED;
            firstFeed = false;
            return;
        }
        feedStage = FeedStage.WINDMILL;
        feedTimer.reset();
        intake.startWindmillOnly();
    }

    private void beginFeed() {
        feedStage = FeedStage.FEED;
        firstFeed = false;
        intake.start();
        intake.startWindmill();
    }

    /**
     * Hands the intake back when feeding ends: if the driver had the intake running before the volley it goes
     * back to intaking (roller in, windmill gate, gentle transfer); otherwise everything stops.
     */
    private void releaseIntake() {
        if (intake != null && feedStage != FeedStage.NONE) {
            intake.stop();
            if (driverIntakeWasOn) intake.start();
        }
        feedStage = FeedStage.NONE;
    }

    /**
     * Estimated number of balls shot since the last {@link #resetShotCount()}: one per flywheel speed drop while
     * feeding (see {@link #SHOT_DROP}). It is an estimate (no ball sensor): a ball that barely slows the flywheel
     * is not counted.
     */
    public int getShotCount() {
        return shotCount;
    }

    public void resetShotCount() {
        shotCount = 0;
    }

    /** @return the current step of the feed sequence, for telemetry. */
    public String getFeedStage() {
        return feedStage.name();
    }
}
