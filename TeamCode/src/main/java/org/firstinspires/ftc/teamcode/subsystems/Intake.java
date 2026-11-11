package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Subsystem controlling the intake roller motor, windmill feeder, and transfer motor.
 *
 * <p>WINDMILL: plain open-loop power (no velocity PID) and FLOAT at zero power (no brake, no position hold).
 * Speeds below are given in motor RPM and converted to power as RPM / {@link #WINDMILL_FREE_RPM}, so the real speed
 * under load is somewhat lower. While the intake runs and the windmill is not feeding, the windmill turns slowly in
 * reverse ({@link #WINDMILL_INTAKE_REVERSE_RPM}) so balls pushed up by the transfer cannot drive it forward.
 *
 * <p>TRANSFER: closed-loop velocity while feeding or ejecting (with soft start, overcurrent back-off and a pause
 * while a feeding windmill is bogged down); gentle open-loop power while only intaking. It brakes at zero power.
 * Every stop path goes through {@link #stopTransfer()}.
 *
 * <p>Shooting order (driven by {@link Shooter}): {@link #clearReverse()} for a short moment, then
 * {@link #startWindmillOnly()}, then {@link #start()} + {@link #startWindmill()} to bring the roller and transfer in.
 * Between shots, while the flywheel recovers, {@link #holdFeed()} closes the gate (windmill stopped with BRAKE,
 * transfer stopped) so the next ball cannot reach a slow flywheel; {@link #resumeWindmill()} reopens it.
 *
 * <p>JAM RECOVERY: if a feeding windmill stays bogged down, the windmill and transfer back up for a moment and
 * try again (see {@link #UNJAM_AFTER_SEC}).
 */
public class Intake {

    // ---------------------- Hardware ----------------------
    private DcMotor intake;
    private DcMotorEx windmill;
    private DcMotorEx transfer;

    // ---------------------- Roller ----------------------
    public static final double INTAKE_POWER = 0.80;
    public static final double INTAKE_IN = -INTAKE_POWER;
    public static final double INTAKE_OUT = INTAKE_POWER;

    // ---------------------- Shared speed units ----------------------
    /** Encoder ticks per motor-shaft revolution for a goBILDA 6000 RPM (1:1) motor. */
    public static final double MOTOR_TICKS_PER_REV = 28.0;
    /** Default feed speed for the transfer and the windmill eject speed, motor RPM. */
    public static final double FEED_RPM = 2000.0;
    /** {@link #FEED_RPM} in ticks/sec: 2000 * 28 / 60 = 933.3. */
    public static final double FEED_TICKS_PER_SEC = FEED_RPM * MOTOR_TICKS_PER_REV / 60.0;

    // ---------------------- Windmill (open loop, no brake) ----------------------
    /** No-load speed of the windmill motor at full power; RPM settings are turned into power with this. */
    public static final double WINDMILL_FREE_RPM = 6000.0;
    /** Reverse speed while intaking (windmill not feeding), so the transfer cannot push it forward. */
    public static final double WINDMILL_INTAKE_REVERSE_RPM = 750.0;
    /** Reverse speed for the short clear before shooting, and for ejecting. */
    public static final double WINDMILL_REVERSE_RPM = FEED_RPM;
    /**
     * While feeding, the windmill runs at this fraction of the flywheel's target speed (both in ticks/sec) unless a
     * fixed feed speed was set. Lower it if balls still pile up.
     */
    public static final double WINDMILL_FEED_RATIO = 0.90;
    /** Highest windmill feed speed, ticks/sec (= full power). */
    public static final double WINDMILL_MAX_FEED_VELOCITY = WINDMILL_FREE_RPM * MOTOR_TICKS_PER_REV / 60.0;
    /**
     * Soft start for forward feeding: power rises at most this much per second, starting from
     * {@link #WINDMILL_START_FRACTION} of the target. Needs {@link #update()} each loop (Shooter and the autonomous
     * feeder call it); without it the windmill stays at the start fraction instead of stopping.
     */
    public static final double WINDMILL_RAMP_POWER_PER_SEC = 4.0;
    public static final double WINDMILL_START_FRACTION = 0.40;

    // Kept so older references still compile (ticks/sec).
    public static final double WINDMILL_VELOCITY = FEED_TICKS_PER_SEC;
    public static final double WINDMILL_IN = WINDMILL_VELOCITY;
    public static final double WINDMILL_OUT = -WINDMILL_VELOCITY;

    // ---------------------- Transfer ----------------------
    /** Signed velocity (ticks/sec) that moves elements INWARD (toward the shooter). */
    public static final double TRANSFER_VELOCITY = -FEED_TICKS_PER_SEC;
    /** Kept so older references still compile; a velocity in ticks/sec. Prefer {@link #TRANSFER_VELOCITY}. */
    public static final double TRANSFER_POWER = TRANSFER_VELOCITY;
    /** Flip this one constant if the transfer motor spins the wrong way on the robot. */
    public static final DcMotorSimple.Direction TRANSFER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    /** Kept so older references still compile; prefer {@link #TRANSFER_VELOCITY}. */
    public static final double TRANSFER_SERVO_POWER = TRANSFER_POWER;
    /**
     * Open-loop power (0..1) for the transfer while intaking with the windmill not feeding. Raise it if balls do
     * not climb the transfer; set 0 to keep the transfer off while intaking.
     */
    public static final double TRANSFER_INTAKE_POWER = 0.25;
    /** Soft start for the transfer velocity (ticks/sec per second) and its starting fraction. */
    public static final double TRANSFER_RAMP_TICKS_PER_SEC2 = 6000;
    public static final double TRANSFER_START_FRACTION = 0.40;
    /**
     * If the transfer draws more than this many amps for {@link #TRANSFER_OVERCURRENT_DELAY_SEC} while feeding or
     * ejecting, it stops for {@link #TRANSFER_COOLDOWN_SEC}, then soft-starts again. Read the real stall number from
     * the test TeleOp's peak current before trusting this value; 0 turns it off.
     */
    public static final double TRANSFER_CURRENT_LIMIT_AMPS = 6.0;
    public static final double TRANSFER_OVERCURRENT_DELAY_SEC = 0.25;
    public static final double TRANSFER_COOLDOWN_SEC = 0.40;
    /**
     * While feeding, if the windmill runs below this fraction of its expected speed for
     * {@link #TRANSFER_PAUSE_DELAY_SEC}, the transfer stops pushing balls in until the windmill is back above
     * {@link #TRANSFER_RESUME_ABOVE}. 0 turns it off. (Expected speed = power x free speed, so it is approximate.)
     */
    public static final double TRANSFER_PAUSE_BELOW = 0.50;
    public static final double TRANSFER_RESUME_ABOVE = 0.75;
    public static final double TRANSFER_PAUSE_DELAY_SEC = 0.12;
    /**
     * GATE BETWEEN SHOTS ({@link #holdFeed()}). While the flywheel recovers after a shot the windmill is stopped
     * with BRAKE so the next ball cannot be pushed through. If balls still slip past a braked windmill, set this
     * above 0 to turn the windmill slowly backwards while holding (the old behaviour used 750); it costs time
     * because the next ball is pushed away from the shooter.
     */
    public static final double WINDMILL_HOLD_REVERSE_RPM = 0.0;
    /**
     * Transfer power (0..1, open loop) while the gate is closed between shots. 0 = stopped, so nothing pushes on
     * the windmill. A small value keeps the next ball pressed against the windmill for a quicker follow-up shot;
     * only raise it if the braked windmill reliably holds the ball back.
     */
    public static final double TRANSFER_HOLD_POWER = 0.0;
    /**
     * JAM RECOVERY. If the transfer has been waiting for a bogged-down windmill for this long, the windmill and
     * transfer run in reverse for {@link #UNJAM_REVERSE_SEC} and then feed again. After {@link #UNJAM_MAX_TRIES}
     * tries in a row without the windmill recovering it stops trying ({@link #isJammed()}). 0 turns it off.
     */
    public static final double UNJAM_AFTER_SEC = 0.30;
    public static final double UNJAM_REVERSE_SEC = 0.10;
    public static final int UNJAM_MAX_TRIES = 3;
    /** Battery voltage the open-loop windmill feed power is referenced to (see {@link #setBatteryVoltage}). */
    public static final double REFERENCE_VOLTAGE = 12.0;
    /** Motor currents are read this often (each read costs a hub round trip). */
    private static final double CURRENT_READ_SEC = 0.05;

    // ---------------------- Live settings (the tuning TeleOp changes these) ----------------------
    private double intakePower = INTAKE_POWER;
    private double windmillFeedVelocity = FEED_TICKS_PER_SEC;          // ticks/sec, open-loop equivalent
    private double windmillOverrideVelocity = Double.NaN;              // NaN = follow the flywheel
    private double windmillIntakeReverseRpm = WINDMILL_INTAKE_REVERSE_RPM;
    private double transferFeedVelocity = TRANSFER_VELOCITY;
    private double transferIntakePower = TRANSFER_INTAKE_POWER;

    // ---------------------- State ----------------------
    private enum WindmillState { OFF, INTAKE_REVERSE, CLEAR_REVERSE, FEED, EJECT, HOLD }
    private enum TransferState { STOPPED, GENTLE, FEED, REVERSE }
    private WindmillState windmillState = WindmillState.OFF;
    private TransferState transferState = TransferState.STOPPED;
    private boolean rollerIn = false;
    private boolean transferOpenLoop = false;
    private double windmillTargetPower = 0, windmillPower = 0;
    private double transferTarget = 0, transferCommand = 0;
    private boolean transferPaused = false;
    private boolean windmillBraking = false;
    private double batteryVoltage = REFERENCE_VOLTAGE;
    private double pausedSeconds = 0, unjamRemaining = 0;
    private int unjamTries = 0, unjamCount = 0;
    private double slowSeconds = 0, overCurrentSeconds = 0, cooldownRemaining = 0, sinceCurrentRead = 0;
    private double windmillAmps = 0, transferAmps = 0, windmillPeakAmps = 0, transferPeakAmps = 0;
    private long lastUpdateNanos = 0;
    private String intakeStatus = "Off";

    /**
     * Initializes the roller, windmill and transfer motors. The windmill is open loop and floats; the transfer
     * starts stopped with BRAKE.
     */
    public void init(HardwareMap hwMap) {
        BulkReads.enable(hwMap);
        intake = hwMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        windmill = hwMap.get(DcMotorEx.class, "windmill");
        windmill.setDirection(DcMotorSimple.Direction.FORWARD);
        // Open loop, no brake: the encoder is still read for telemetry and the transfer pause, never for control.
        windmill.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        windmill.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        windmillState = WindmillState.OFF;
        windmillTargetPower = windmillPower = 0;
        windmill.setPower(0);

        transfer = hwMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(TRANSFER_DIRECTION);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferOpenLoop = false;
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rollerIn = false;
        stopTransfer();
    }

    // =====================================================================
    // Public actions
    // =====================================================================

    /**
     * Runs the roller inwards and the transfer inward: full feed speed if the windmill is feeding, otherwise gently
     * with the windmill turning slowly in reverse.
     */
    public void start() {
        intakeStatus = "Running Inwards";
        intake.setPower(-intakePower);
        rollerIn = true;
        transferInward();
    }

    /** Runs the roller, windmill, and transfer in reverse to eject game elements. */
    public void out() {
        intakeStatus = "Running Reverse";
        intake.setPower(intakePower);
        rollerIn = false;
        setWindmill(WindmillState.EJECT);
        transferAtVelocity(-transferFeedVelocity, TransferState.REVERSE);
    }

    /** Stops the roller, windmill and transfer. */
    public void stop() {
        stopRoller();
        setWindmill(WindmillState.OFF);
        stopTransfer();
    }

    /** Stops only the roller. A windmill that was only turning slowly backwards for intaking stops too. */
    public void stopRoller() {
        intakeStatus = "Off";
        intake.setPower(0);
        rollerIn = false;
        if (windmillState == WindmillState.INTAKE_REVERSE) setWindmill(WindmillState.OFF);
    }

    /** Sets a custom roller speed. Positive = inwards, negative = reverse (ejects). */
    public void switchSpeed(double speedInput) {
        double speed = Math.max(-1.0, Math.min(1.0, speedInput));
        if (speed > 0) {
            intakeStatus = String.format(java.util.Locale.US, "Running Inwards at (%.2f)", speed);
            intake.setPower(-speed);
            rollerIn = true;
            transferInward();
        } else if (speed < 0) {
            intakeStatus = String.format(java.util.Locale.US, "Running Reverse at (%.2f)", speed);
            intake.setPower(-speed);
            rollerIn = false;
            setWindmill(WindmillState.EJECT);
            transferAtVelocity(-transferFeedVelocity, TransferState.REVERSE);
        } else {
            stop();
        }
    }

    /** Windmill forward at the feed speed AND the transfer at full feed speed. */
    public void startWindmill() {
        setWindmill(WindmillState.FEED);
        transferAtVelocity(transferFeedVelocity, TransferState.FEED);
    }

    /**
     * Closes the gate between shots: the windmill stops and BRAKES (or turns slowly backwards if
     * {@link #WINDMILL_HOLD_REVERSE_RPM} is set) and the transfer stops, so the next ball stays where it is until
     * the flywheel is back at speed. The roller is left as it was. Reopen with {@link #resumeWindmill()}.
     */
    public void holdFeed() {
        intakeStatus = "Holding (flywheel recovering)";
        setWindmill(WindmillState.HOLD);
        if (TRANSFER_HOLD_POWER > 0) transferGentle(TRANSFER_HOLD_POWER);
        else stopTransfer();
    }

    /**
     * Reopens the gate after {@link #holdFeed()}: the windmill goes straight to its feed power (no soft start, it
     * was only stopped for a moment) and the transfer soft-starts behind it. Call {@link #start()} afterwards to
     * bring the roller back in.
     */
    public void resumeWindmill() {
        setWindmill(WindmillState.FEED);
        if (unjamRemaining <= 0) {
            windmillPower = windmillTargetPower;
            windmill.setPower(windmillPower);
        }
        transferAtVelocity(transferFeedVelocity, TransferState.FEED);
    }

    /** @return true while the gate is closed by {@link #holdFeed()}. */
    public boolean isHoldingFeed() { return windmillState == WindmillState.HOLD; }

    /** @return true while the roller is running inward (intaking or feeding). */
    public boolean isRollerIn() { return rollerIn; }

    /** Shooting step 2: windmill forward at the feed speed only; roller and transfer stay off. */
    public void startWindmillOnly() {
        intakeStatus = "Windmill only";
        intake.setPower(0);
        rollerIn = false;
        setWindmill(WindmillState.FEED);
        stopTransfer();
    }

    /** Shooting step 1: windmill and transfer in reverse to unload them; the roller stays off. */
    public void clearReverse() {
        intakeStatus = "Clearing";
        intake.setPower(0);
        rollerIn = false;
        setWindmill(WindmillState.CLEAR_REVERSE);
        transferAtVelocity(-transferFeedVelocity, TransferState.REVERSE);
    }

    /**
     * Stops windmill feeding. With the roller still running inward the windmill goes back to its slow reverse;
     * otherwise it stops (floating). A transfer that was feeding drops to its gentle intake power.
     */
    public void stopWindmill() {
        setWindmill(rollerIn ? WindmillState.INTAKE_REVERSE : WindmillState.OFF);
        if (transferState == TransferState.FEED) transferGentle();
    }

    public void startTransfer() {
        transferAtVelocity(transferFeedVelocity, TransferState.FEED);
    }

    /** Stops the transfer motor and re-asserts BRAKE so it cannot be left coasting. */
    public void stopTransfer() {
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setPower(0.0);
        transferState = TransferState.STOPPED;
        transferCommand = transferTarget = 0;
        cooldownRemaining = overCurrentSeconds = 0;
        transferPaused = false;
        slowSeconds = 0;
        pausedSeconds = 0;
    }

    /** @deprecated the transfer is a motor now; use {@link #startTransfer()}. */
    @Deprecated
    public void startTransferServo() { startTransfer(); }

    /** @deprecated the transfer is a motor now; use {@link #stopTransfer()}. */
    @Deprecated
    public void stopTransferServo() { stopTransfer(); }

    public String getIntakeStatus() { return intakeStatus; }

    // =====================================================================
    // Live settings
    // =====================================================================

    /** Roller power 0..1 used when intaking inward or ejecting. */
    public void setIntakePower(double power) { intakePower = Math.max(0, Math.min(1, power)); }
    public double getIntakePower() { return intakePower; }

    /** Fixed windmill feed speed in motor RPM (open loop). Pass NaN to go back to following the flywheel. */
    public void setWindmillFeedRpm(double rpm) {
        if (Double.isNaN(rpm)) { windmillOverrideVelocity = Double.NaN; return; }
        windmillOverrideVelocity = Math.max(0, Math.min(WINDMILL_MAX_FEED_VELOCITY, rpm * MOTOR_TICKS_PER_REV / 60.0));
        windmillFeedVelocity = windmillOverrideVelocity;
    }
    public boolean isWindmillFollowingFlywheel() { return Double.isNaN(windmillOverrideVelocity); }
    public double getWindmillFeedRpm() { return windmillFeedVelocity * 60.0 / MOTOR_TICKS_PER_REV; }

    /** Windmill reverse speed while intaking, motor RPM (open loop). */
    public void setWindmillIntakeReverseRpm(double rpm) {
        windmillIntakeReverseRpm = Math.max(0, Math.min(WINDMILL_FREE_RPM, rpm));
        if (windmillState == WindmillState.INTAKE_REVERSE) setWindmill(WindmillState.INTAKE_REVERSE);
    }
    public double getWindmillIntakeReverseRpm() { return windmillIntakeReverseRpm; }

    /** Transfer full feed speed in motor RPM (direction stays the same as TRANSFER_VELOCITY). */
    public void setTransferFeedRpm(double rpm) {
        double ticks = Math.max(0, Math.min(WINDMILL_MAX_FEED_VELOCITY, rpm * MOTOR_TICKS_PER_REV / 60.0));
        transferFeedVelocity = Math.signum(TRANSFER_VELOCITY) * ticks;
    }
    public double getTransferFeedRpm() { return Math.abs(transferFeedVelocity) * 60.0 / MOTOR_TICKS_PER_REV; }

    /** Open-loop transfer power 0..1 while intaking with the windmill not feeding. */
    public void setTransferIntakePower(double power) { transferIntakePower = Math.max(0, Math.min(1, power)); }
    public double getTransferIntakePower() { return transferIntakePower; }

    /**
     * Sets the windmill feed speed from the flywheel target (ticks/sec): {@link #WINDMILL_FEED_RATIO} times the
     * target, unless a fixed feed speed was set.
     */
    public void setWindmillFeedFromFlywheel(double flywheelTargetTicksPerSec) {
        if (!Double.isNaN(windmillOverrideVelocity)) return;
        windmillFeedVelocity = flywheelTargetTicksPerSec > 0
                ? Math.min(WINDMILL_MAX_FEED_VELOCITY, WINDMILL_FEED_RATIO * flywheelTargetTicksPerSec)
                : FEED_TICKS_PER_SEC;
        if (windmillState == WindmillState.FEED) windmillTargetPower = windmillPowerFor(WindmillState.FEED);
    }

    /**
     * Tells the intake the battery voltage (Shooter passes its cached reading every loop). The windmill is open
     * loop, so its feed power is scaled by {@link #REFERENCE_VOLTAGE} / voltage to keep the same speed as the
     * battery drains. Never called = no scaling.
     */
    public void setBatteryVoltage(double volts) {
        if (Double.isNaN(volts) || volts < 6 || volts > 15) return;
        batteryVoltage = volts;
        if (windmillState == WindmillState.FEED) windmillTargetPower = windmillPowerFor(WindmillState.FEED);
    }

    // =====================================================================
    // Per-loop update
    // =====================================================================

    /**
     * Call once per loop (Shooter.update/updateSpinupOnly and the autonomous feeder already do). Runs the soft
     * starts, the transfer overcurrent back-off and pause, and samples motor currents.
     */
    public void update() {
        long now = System.nanoTime();
        double dt = lastUpdateNanos == 0 ? 0 : Math.min(0.1, (now - lastUpdateNanos) / 1e9);
        lastUpdateNanos = now;
        if (windmill == null) return;

        // Windmill soft start (forward feeding only; reverse speeds are small and applied directly).
        if (unjamRemaining <= 0 && windmillPower != windmillTargetPower) {
            double step = WINDMILL_RAMP_POWER_PER_SEC * dt;
            double diff = windmillTargetPower - windmillPower;
            windmillPower = Math.abs(diff) <= step ? windmillTargetPower : windmillPower + Math.signum(diff) * step;
            windmill.setPower(windmillPower);
        }

        updateTransferRamp(dt);
        sinceCurrentRead += dt;
        if (sinceCurrentRead >= CURRENT_READ_SEC) {
            double readDt = sinceCurrentRead;
            sinceCurrentRead = 0;
            windmillAmps = windmill.getCurrent(CurrentUnit.AMPS);
            transferAmps = transfer.getCurrent(CurrentUnit.AMPS);
            windmillPeakAmps = Math.max(windmillPeakAmps, windmillAmps);
            transferPeakAmps = Math.max(transferPeakAmps, transferAmps);
            updateTransferOvercurrent(readDt);
        }
        updateTransferGate(dt);
    }

    // ---------------------- Telemetry getters ----------------------
    public double getWindmillCurrentAmps() { return windmillAmps; }
    public double getWindmillPeakCurrentAmps() { return windmillPeakAmps; }
    public double getTransferCurrentAmps() { return transferAmps; }
    public double getTransferPeakCurrentAmps() { return transferPeakAmps; }
    public void resetPeakCurrent() { windmillPeakAmps = 0; transferPeakAmps = 0; }
    /** Windmill measured speed in motor RPM (encoder), negative when reversing. */
    public double getWindmillMeasuredRpm() { return windmill == null ? 0 : windmill.getVelocity() * 60.0 / MOTOR_TICKS_PER_REV; }
    /** Windmill power currently applied (-1..1). */
    public double getWindmillPower() { return windmillPower; }
    public boolean isTransferPaused() { return transferPaused; }
    /** @return true while the windmill and transfer are backing up to free a jam. */
    public boolean isUnjamming() { return unjamRemaining > 0; }
    /** @return true when jam recovery has given up; reverse the intake (eject) or stop to clear it. */
    public boolean isJammed() { return transferPaused && unjamRemaining <= 0 && unjamTries >= UNJAM_MAX_TRIES; }
    /** @return how many times jam recovery has run since init (for telemetry / tuning). */
    public int getUnjamCount() { return unjamCount; }
    public boolean isTransferOvercurrentCooling() { return cooldownRemaining > 0; }

    // =====================================================================
    // Windmill helpers
    // =====================================================================

    private double windmillPowerFor(WindmillState state) {
        switch (state) {
            case INTAKE_REVERSE: return -windmillIntakeReverseRpm / WINDMILL_FREE_RPM;
            case CLEAR_REVERSE:
            case EJECT:          return -WINDMILL_REVERSE_RPM / WINDMILL_FREE_RPM;
            case FEED:           return Math.min(1.0, windmillFeedVelocity / WINDMILL_MAX_FEED_VELOCITY
                                         * REFERENCE_VOLTAGE / batteryVoltage);
            case HOLD:           return -WINDMILL_HOLD_REVERSE_RPM / WINDMILL_FREE_RPM;
            default:             return 0;
        }
    }

    private void setWindmill(WindmillState state) {
        if (state == WindmillState.FEED && unjamRemaining > 0) return;   // jam recovery owns the windmill for a moment
        unjamRemaining = 0;
        // Brake only while holding the gate closed; everywhere else the windmill floats as before.
        boolean brake = state == WindmillState.HOLD;
        if (brake != windmillBraking) {
            windmill.setZeroPowerBehavior(brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
            windmillBraking = brake;
        }
        if (state != WindmillState.FEED && state != WindmillState.HOLD) unjamTries = 0;
        double target = windmillPowerFor(state);
        if (state == WindmillState.FEED) {
            // Soft start: from rest or reverse, begin at a fraction of the feed power and ramp up in update().
            if (windmillState != WindmillState.FEED || windmillPower <= 0) {
                windmillPower = WINDMILL_START_FRACTION * target;
                if (lastUpdateNanos == 0) lastUpdateNanos = System.nanoTime();
            }
        } else {
            windmillPower = target;   // stop and the small reverse speeds apply immediately
        }
        windmillState = state;
        windmillTargetPower = target;
        windmill.setPower(windmillPower);
        if (state != WindmillState.FEED) { transferPaused = false; slowSeconds = 0; pausedSeconds = 0; }
    }

    // =====================================================================
    // Transfer helpers
    // =====================================================================

    private void transferInward() {
        if (windmillState == WindmillState.FEED) {
            transferAtVelocity(transferFeedVelocity, TransferState.FEED);
        } else {
            setWindmill(WindmillState.INTAKE_REVERSE);
            transferGentle();
        }
    }

    private boolean transferClosedLoopRunning() {
        return !transferOpenLoop && (transferState == TransferState.FEED || transferState == TransferState.REVERSE);
    }

    private boolean transferHeldBack() {
        return cooldownRemaining > 0 || (transferState == TransferState.FEED && transferPaused);
    }

    /** Closed-loop transfer velocity (ticks/sec) with soft start. */
    private void transferAtVelocity(double ticksPerSec, TransferState state) {
        if (state == TransferState.FEED && unjamRemaining > 0) return;   // jam recovery owns the transfer for a moment
        if (transferOpenLoop) {
            transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            transferOpenLoop = false;
        }
        transferTarget = ticksPerSec;
        if (transferCommand == 0 || Math.signum(transferCommand) != Math.signum(ticksPerSec)) {
            transferCommand = TRANSFER_START_FRACTION * ticksPerSec;
            if (lastUpdateNanos == 0) lastUpdateNanos = System.nanoTime();
        }
        transferState = state;
        transfer.setVelocity(transferHeldBack() ? 0 : transferCommand);
    }

    /**
     * Open-loop low power inward. Open loop on purpose: a velocity PID would ramp to full power when a ball
     * stalls against the windmill.
     */
    private void transferGentle() { transferGentle(transferIntakePower); }

    private void transferGentle(double power) {
        if (!transferOpenLoop) {
            transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transferOpenLoop = true;
        }
        transfer.setPower(Math.signum(TRANSFER_VELOCITY) * power);
        transferState = TransferState.GENTLE;
        transferCommand = transferTarget = 0;
        cooldownRemaining = overCurrentSeconds = 0;
        transferPaused = false;
    }

    private void updateTransferRamp(double dt) {
        if (!transferClosedLoopRunning()) return;
        if (cooldownRemaining > 0) {
            cooldownRemaining -= dt;
            if (cooldownRemaining <= 0) {
                cooldownRemaining = 0;
                transferCommand = TRANSFER_START_FRACTION * transferTarget;
                transfer.setVelocity(transferHeldBack() ? 0 : transferCommand);
            }
            return;
        }
        if (transferHeldBack() || transferCommand == transferTarget) return;
        double step = TRANSFER_RAMP_TICKS_PER_SEC2 * dt;
        double diff = transferTarget - transferCommand;
        transferCommand = Math.abs(diff) <= step ? transferTarget : transferCommand + Math.signum(diff) * step;
        transfer.setVelocity(transferCommand);
    }

    private void updateTransferOvercurrent(double readDt) {
        boolean armed = TRANSFER_CURRENT_LIMIT_AMPS > 0 && transferClosedLoopRunning() && !transferHeldBack()
                && Math.abs(transferCommand) >= 0.9 * Math.abs(transferTarget);   // ignore the start-up surge
        if (!armed || transferAmps <= TRANSFER_CURRENT_LIMIT_AMPS) { overCurrentSeconds = 0; return; }
        overCurrentSeconds += readDt;
        if (overCurrentSeconds >= TRANSFER_OVERCURRENT_DELAY_SEC) {
            overCurrentSeconds = 0;
            cooldownRemaining = TRANSFER_COOLDOWN_SEC;
            transferCommand = 0;
            transfer.setVelocity(0);
        }
    }

    /** Pauses the transfer while a feeding windmill is bogged down, and backs both up if it stays bogged down. */
    private void updateTransferGate(double dt) {
        if (unjamRemaining > 0) {
            unjamRemaining -= dt;
            if (unjamRemaining <= 0) endUnjam();
            return;
        }
        boolean active = TRANSFER_PAUSE_BELOW > 0 && windmillState == WindmillState.FEED
                && transferState == TransferState.FEED && windmillTargetPower > 0
                && windmillPower >= 0.95 * windmillTargetPower;   // ignore the soft-start ramp
        if (!active) {
            slowSeconds = 0;
            pausedSeconds = 0;
            if (transferPaused) { transferPaused = false; restartTransferSoft(); }
            return;
        }
        // Expected speed follows the battery: the same power turns the windmill slower on a low battery.
        double expected = windmillPower * WINDMILL_MAX_FEED_VELOCITY * batteryVoltage / REFERENCE_VOLTAGE;
        double ratio = windmill.getVelocity() / expected;
        if (!transferPaused) {
            slowSeconds = ratio < TRANSFER_PAUSE_BELOW ? slowSeconds + dt : 0;
            if (slowSeconds >= TRANSFER_PAUSE_DELAY_SEC) {
                transferPaused = true;
                pausedSeconds = 0;
                transferCommand = 0;
                transfer.setVelocity(0);
            }
        } else if (ratio > TRANSFER_RESUME_ABOVE) {
            transferPaused = false;
            slowSeconds = 0;
            pausedSeconds = 0;
            unjamTries = 0;
            restartTransferSoft();
        } else {
            pausedSeconds += dt;
            if (UNJAM_AFTER_SEC > 0 && pausedSeconds >= UNJAM_AFTER_SEC && unjamTries < UNJAM_MAX_TRIES) startUnjam();
        }
    }

    /** Backs the windmill and transfer up for a moment to free a ball wedged at the windmill. */
    private void startUnjam() {
        unjamTries++;
        unjamCount++;
        unjamRemaining = UNJAM_REVERSE_SEC;
        pausedSeconds = 0;
        windmillPower = -WINDMILL_REVERSE_RPM / WINDMILL_FREE_RPM;
        windmill.setPower(windmillPower);
        transferCommand = 0;
        transfer.setVelocity(-transferFeedVelocity);
    }

    /** After the back-up: soft-start the windmill forward again and let the transfer follow. */
    private void endUnjam() {
        unjamRemaining = 0;
        if (windmillState != WindmillState.FEED) return;
        windmillPower = WINDMILL_START_FRACTION * windmillTargetPower;
        windmill.setPower(windmillPower);
        transferPaused = false;
        slowSeconds = 0;
        pausedSeconds = 0;
        if (transferState == TransferState.FEED) restartTransferSoft(); else transfer.setVelocity(0);
    }

    private void restartTransferSoft() {
        if (transferState != TransferState.FEED || cooldownRemaining > 0) return;
        transferTarget = transferFeedVelocity;
        transferCommand = TRANSFER_START_FRACTION * transferFeedVelocity;
        transfer.setVelocity(transferCommand);
    }
}
