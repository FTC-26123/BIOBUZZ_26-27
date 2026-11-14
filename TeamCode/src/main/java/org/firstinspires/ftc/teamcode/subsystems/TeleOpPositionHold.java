package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Idle-only pose hold using the real robot's existing Pedro Foresight controllers. */
public final class TeleOpPositionHold {
    /** Stick values smaller than this count as zero, per axis (a worn stick resting at 0.06-0.08 must not turn the robot). */
    public static final double STICK_DEADBAND = 0.08;

    /** The stick value with the dead zone removed (0 inside it), so a resting stick never moves the robot. */
    public static double deadband(double v) {
        return !Double.isFinite(v) || Math.abs(v) <= STICK_DEADBAND ? 0 : v;
    }
    public static final double ENGAGE_DELAY_SECONDS = 0.15;
    public static double MAX_HOLD_POWER = 1.00;
    /**
     * Integral term added on top of Pedro's hold. A steady push makes the extra power grow over time, so the robot
     * is walked back to its spot instead of settling short. HOLD_KI is how fast it grows (0 turns it off); the
     * extra power from it never exceeds HOLD_I_LIMIT_POWER per axis. The stored integral is cleared whenever the
     * hold releases or faults, and drains quickly when a push ends so it does not overshoot.
     */
    public static double HOLD_KI = 1.0;
    public static final double HOLD_I_LIMIT_POWER = 0.35;
    public static final double HOLD_I_DRAIN_PER_S = 5.0;
    /**
     * Damping: extra power proportional to how fast Pedro's correction is changing, which reacts to the speed of a
     * shove and calms shaking from the stiff gains. HOLD_KD_SECONDS is the look-ahead time (0 turns it off);
     * HOLD_D_FILTER smooths the signal (1.0 = no smoothing).
     */
    public static double HOLD_KD_SECONDS = 0.05;
    public static final double HOLD_D_FILTER = 0.3;
    /** Most extra power the damping term may add per axis (Pedro's output jumps at its 2.5 in gain switch). */
    public static final double HOLD_D_LIMIT_POWER = 0.20;

    /**
     * Integral and damping added on top of Pedro's hold output, per axis in Pedro's own robot-frame powers.
     * Shared by the TeleOp hold and the hold tuner so both behave the same.
     */
    public static final class HoldExtras {
        public double kI = HOLD_KI, kD = HOLD_KD_SECONDS;
        private double iF, iS, iT, pF, pS, pT, dF, dS, dT;
        private boolean havePrevious;

        public void reset() {
            iF = iS = iT = dF = dS = dT = 0;
            havePrevious = false;
        }

        private double integrate(double integral, double power, double dt) {
            if (kI <= 0) return 0;
            // A correction that opposes the stored integral means the push ended: drain it fast to avoid overshoot.
            if (integral * power < 0) integral *= Math.max(0, 1 - HOLD_I_DRAIN_PER_S * dt);
            integral += power * dt;
            double limit = HOLD_I_LIMIT_POWER / kI;
            return Math.max(-limit, Math.min(limit, integral));
        }

        private static double clampD(double v) {
            return Math.max(-HOLD_D_LIMIT_POWER, Math.min(HOLD_D_LIMIT_POWER, v));
        }

        public DrivePowers apply(DrivePowers p, double dt) {
            iF = integrate(iF, p.forward(), dt);
            iS = integrate(iS, p.strafe(), dt);
            iT = integrate(iT, p.turn(), dt);
            if (havePrevious && dt > 1e-4) {
                dF += HOLD_D_FILTER * ((p.forward() - pF) / dt - dF);
                dS += HOLD_D_FILTER * ((p.strafe() - pS) / dt - dS);
                dT += HOLD_D_FILTER * ((p.turn() - pT) / dt - dT);
            }
            pF = p.forward(); pS = p.strafe(); pT = p.turn();
            havePrevious = true;
            return new DrivePowers(
                    p.forward() + kI * iF + clampD(kD * dF),
                    p.strafe() + kI * iS + clampD(kD * dS),
                    p.turn() + kI * iT + clampD(kD * dT));
        }
    }
    private static final double MAX_LOOP_GAP_SECONDS = 0.25;
    /**
     * Keep resisting a shove up to this far from the captured target. Beyond it the robot was carried somewhere
     * else entirely, so driving back could ram something; the hold stops and the driver re-arms with a stick.
     */
    public static final double MAX_HOLD_RANGE_INCHES = 36.0;
    /**
     * The hold gives up if the robot is turned further than this from its held heading. A shove rarely turns the robot
     * more than this; a hold turning the WRONG way (heading direction reversed) passes it in a fraction of a second,
     * so this is what stops a runaway spin quickly. (Was 120 degrees: a wrong-way hold could spin a third of a turn.)
     */
    public static double MAX_HOLD_HEADING = Math.toRadians(30);
    /** Heading runaways in one OpMode before the hold switches itself off for the rest of it. */
    public static final int MAX_HEADING_RUNAWAYS = 2;
    /**
     * A real push cannot move the robot faster than this between two updates. A bigger step means the odometry
     * jumped (reset, glitch, re-localization), and holding would chase a phantom target.
     */
    public static final double MAX_PUSH_SPEED_IN_PER_S = 120.0;
    public static final double MAX_PUSH_TURN_RAD_PER_S = Math.toRadians(720);
    private static final double MIN_JUMP_INCHES = 4.0, MIN_JUMP_RADIANS = Math.toRadians(15);

    // Keeps ownership/transition tests independent of Android; production uses PedroBackend below.
    public interface Backend {
        Pose updatePose();
        void hold(Pose target, double dt);
        /** Hold x/y (and heading unless overridden); turnPower = turn power to use instead (NaN = hold heading). */
        default void hold(Pose target, double dt, double turnPower) { hold(target, dt); }
        void stop();
        /** Re-read the hold settings (stiffness) after they were changed. */
        default void refresh() { }
    }

    private final Backend backend;
    private Pose target;
    private double idleSince = Double.NaN;
    private double previousTime = Double.NaN;
    private Pose previousPose;
    private boolean faulted;
    private String status = "Ready";
    private double offsetNow = 0, peakOffset = 0;
    /** The hold only arms after the driver has moved a stick once, so nothing moves the robot right after Play. */
    private boolean driverHasDriven = false;
    private int headingRunaways = 0;
    private boolean disabled = false;

    public TeleOpPositionHold(Backend backend) { this.backend = backend; }

    public static TeleOpPositionHold create(HardwareMap hardwareMap) {
        RobotSettings.ensureLoaded();
        return new TeleOpPositionHold(new PedroBackend(hardwareMap));
    }

    public static boolean sticksIdle(double y, double x, double rx) {
        return Double.isFinite(y) && Double.isFinite(x) && Double.isFinite(rx)
                && Math.abs(y) <= STICK_DEADBAND && Math.abs(x) <= STICK_DEADBAND
                && Math.abs(rx) <= STICK_DEADBAND;
    }

    /** Returns true when hold owns the motors, including zero power while arming/faulted. */
    public boolean update(double y, double x, double rx, boolean bypass, double now) {
        if (disabled) { status = "OFF: heading kept running away - check the Pinpoint heading (Tune Position Hold)"; return false; }
        if (!sticksIdle(y, x, rx)) driverHasDriven = true;
        if (!driverHasDriven) { status = "Waits for the first drive"; return false; }
        if (bypass || !sticksIdle(y, x, rx)) {
            release();
            status = bypass ? "Bypassed" : "Driving";
            return false;
        }
        return step(now, Double.NaN, false);
    }

    /**
     * AIM held: keep the robot's x/y locked (it cannot be pushed off its spot) while the heading turns to the
     * camera's aim point. Moving the left stick drives normally (returns false; the caller aims with the stick
     * turning taken over by the camera).
     *
     * @param turnPowerRight auto-rotate turn power (positive = clockwise), from the aim PD; NaN = no target this
     *                       loop (hold the current heading)
     */
    public boolean updateAiming(double y, double x, double turnPowerRight, double now) {
        if (disabled) { status = "OFF: heading kept running away - check the Pinpoint heading (Tune Position Hold)"; return false; }
        driverHasDriven = true;   // AIM is a driver action
        if (!sticksIdle(y, x, 0)) {
            release();
            status = "Driving (aim)";
            return false;
        }
        return step(now, turnPowerRight, true);
    }

    private boolean step(double now, double turnPowerRight, boolean aiming) {
        if (faulted) { backend.stop(); return true; }
        double dt = Double.isNaN(previousTime) ? 0 : now - previousTime;
        previousTime = now;
        if (!Double.isFinite(now) || dt < 0 || dt > MAX_LOOP_GAP_SECONDS) {
            fail("Loop interrupted - move stick to rearm");
            return true;
        }
        try {
            Pose pose = backend.updatePose();
            if (!finite(pose)) throw new IllegalStateException("Invalid pose");
            if (previousPose != null) {
                double step = previousPose.distance(pose);
                double turn = Math.abs(Math.atan2(Math.sin(pose.heading() - previousPose.heading()),
                        Math.cos(pose.heading() - previousPose.heading())));
                if (step > Math.max(MIN_JUMP_INCHES, MAX_PUSH_SPEED_IN_PER_S * dt)
                        || turn > Math.max(MIN_JUMP_RADIANS, MAX_PUSH_TURN_RAD_PER_S * dt)) {
                    fail("Odometry jump - move stick to rearm");
                    return true;
                }
            }
            previousPose = pose;
            if (Double.isNaN(idleSince)) idleSince = now;
            if (now - idleSince < ENGAGE_DELAY_SECONDS) {
                backend.stop();
                status = "Arming";
                return true;
            }
            if (target == null) target = pose;
            boolean override = aiming && !Double.isNaN(turnPowerRight);
            if (override) {
                // The aim PD does the turning; the hold keeps x/y. Its heading target follows the robot so it
                // does not fight the turn, and is frozen wherever the robot is when the target is lost.
                target = new Pose(target.x(), target.y(), pose.heading());
            }
            double headingError = Math.atan2(Math.sin(target.heading() - pose.heading()),
                    Math.cos(target.heading() - pose.heading()));
            // Pushes are resisted at any distance up to the hold range; only a carry far past it stops the hold.
            double offset = target.distance(pose);
            offsetNow = offset;
            peakOffset = Math.max(peakOffset, offset);
            if (!aiming && Math.abs(headingError) > MAX_HOLD_HEADING) {
                // Either a big shove, or the hold turned the wrong way. Stop turning now; after repeated runaways
                // switch the hold off for the rest of the OpMode so it can never spin the robot.
                if (++headingRunaways >= MAX_HEADING_RUNAWAYS) disabled = true;
                fail(disabled ? "OFF: heading kept running away - check the Pinpoint heading (Tune Position Hold)"
                        : "Turned " + Math.round(Math.toDegrees(Math.abs(headingError))) + " deg off - move stick to rearm");
                return true;
            }
            if (offset > MAX_HOLD_RANGE_INCHES) {
                fail("Pushed beyond hold range - move stick to rearm");
                return true;
            }
            // Pedro's turn power is counter-clockwise positive; the aim PD's is clockwise positive.
            if (override) backend.hold(target, dt, -turnPowerRight); else backend.hold(target, dt);
            status = aiming ? (offset > 1.0 ? "Aiming, resisting push" : "Aiming, spot locked")
                    : offset > 1.0 || Math.abs(headingError) > Math.toRadians(3) ? "Resisting push" : "Holding";
        } catch (RuntimeException e) {
            fail("Odometry/hold fault - move stick to rearm");
        }
        return true;
    }

    private static boolean finite(Pose p) {
        return p != null && Double.isFinite(p.x()) && Double.isFinite(p.y())
                && Double.isFinite(p.heading());
    }

    private void fail(String reason) {
        target = null;
        previousPose = null;
        faulted = true;
        status = reason;
        backend.stop();
    }

    public void release() {
        if (!Double.isNaN(idleSince) || target != null || faulted) backend.stop();
        target = null;
        idleSince = Double.NaN;
        previousTime = Double.NaN;
        previousPose = null;
        faulted = false;
        status = "Ready";
    }

    public String getStatus() { return status; }

    /** How far the robot is from its held spot right now (inches; 0 when not holding). */
    public double getOffsetInches() { return target == null ? 0 : offsetNow; }

    /** Largest push seen since {@link #resetPeak()} (inches). */
    public double getPeakOffsetInches() { return peakOffset; }

    public void resetPeak() { peakOffset = 0; }

    /** Apply changed hold settings (stiffness rebuilds the controller; KI, KD and max power are read live). */
    public void refreshTuning() { backend.refresh(); }

    static DrivePowers limitPowers(DrivePowers powers) {
        double forward = powers.forward(), strafe = powers.strafe(), turn = powers.turn();
        if (!Double.isFinite(forward) || !Double.isFinite(strafe) || !Double.isFinite(turn)) {
            throw new IllegalStateException("Invalid hold output");
        }
        double sum = Math.abs(forward) + Math.abs(strafe) + Math.abs(turn);
        if (!Double.isFinite(sum)) throw new IllegalStateException("Invalid hold output magnitude");
        double scale = Math.max(1, sum / MAX_HOLD_POWER);
        return new DrivePowers(forward / scale, strafe / scale, turn / scale);
    }

    /** Same localizer, drivetrain and Foresight tuning as autonomous, with the TeleOp hold stiffness. */
    private static final class PedroBackend implements Backend {
        private Follower follower;
        private final GoBildaPinpointDriver pinpoint;
        private final DcMotor[] motors;

        PedroBackend(HardwareMap hardwareMap) {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
            motors = new DcMotor[] {
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontLeftName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontRightName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backLeftName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backRightName.get())
            };
            follower = Constants.createTeleOpHold(hardwareMap);
            stop();
        }

        @Override public Pose updatePose() {
            // Read once, then validate BEFORE any controller or motor write.
            follower.localizer.update();
            if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                throw new IllegalStateException("Pinpoint not ready");
            }
            Velocity velocity = follower.velocity();
            if (!Double.isFinite(velocity.vx) || !Double.isFinite(velocity.vy)
                    || !Double.isFinite(velocity.omega)) {
                throw new IllegalStateException("Invalid velocity");
            }
            return follower.pose();
        }

        private final HoldExtras extras = new HoldExtras();

        @Override public void hold(Pose target, double dt, double turnPower) {
            extras.kI = HOLD_KI;
            extras.kD = HOLD_KD_SECONDS;
            DrivePowers powers = follower.algorithm().calculateHold(follower.drivetrain,
                    target, follower.localizer.state(), true, dt);
            powers = extras.apply(powers, dt);
            if (!Double.isNaN(turnPower)) powers = new DrivePowers(powers.forward(), powers.strafe(), turnPower);
            follower.drivetrain.drive(limitPowers(powers), false);
        }

        @Override public void hold(Pose target, double dt) {
            // Same operation as Follower HOLD; split out so sensor validation precedes output.
            // true applies the TeleOp hold stiffness (Constants.TELEOP_HOLD_STIFFNESS).
            extras.kI = HOLD_KI;          // read live so tuning applies at once
            extras.kD = HOLD_KD_SECONDS;
            DrivePowers powers = follower.algorithm().calculateHold(follower.drivetrain,
                    target, follower.localizer.state(), true, dt);
            // L1 bound keeps every mixed wheel <= the hold power limit, preserving ratios.
            follower.drivetrain.drive(limitPowers(extras.apply(powers, dt)), false);
        }

        @Override public void refresh() {
            follower = new Follower(follower.localizer, follower.drivetrain,
                    new com.pedropathing.algorithm.Foresight(Constants.holdConfig(Constants.TELEOP_HOLD_STIFFNESS)));
            extras.reset();
        }

        @Override public void stop() {
            extras.reset();
            follower.stop();
            // Follower.stop() only changes mode; force the actual motor zero immediately.
            follower.drivetrain.stop(true);
            // Manual TeleOp also writes these motors, so Pedro's cache alone cannot zero them.
            for (DcMotor motor : motors) motor.setPower(0);
            follower.algorithm().reset();
        }
    }
}
