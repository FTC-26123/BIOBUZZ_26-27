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
    public static final double STICK_DEADBAND = 0.06;
    public static final double ENGAGE_DELAY_SECONDS = 0.15;
    public static final double MAX_HOLD_POWER = 0.90;
    private static final double MAX_LOOP_GAP_SECONDS = 0.25;
    private static final double MAX_DISPLACEMENT_INCHES = 12.0;
    private static final double MAX_HEADING_ERROR = Math.toRadians(45);

    // Keeps ownership/transition tests independent of Android; production uses PedroBackend below.
    public interface Backend {
        Pose updatePose();
        void hold(Pose target, double dt);
        void stop();
    }

    private final Backend backend;
    private Pose target;
    private double idleSince = Double.NaN;
    private double previousTime = Double.NaN;
    private boolean faulted;
    private String status = "Ready";

    public TeleOpPositionHold(Backend backend) { this.backend = backend; }

    public static TeleOpPositionHold create(HardwareMap hardwareMap) {
        return new TeleOpPositionHold(new PedroBackend(hardwareMap));
    }

    public static boolean sticksIdle(double y, double x, double rx) {
        return Double.isFinite(y) && Double.isFinite(x) && Double.isFinite(rx)
                && Math.abs(y) <= STICK_DEADBAND && Math.abs(x) <= STICK_DEADBAND
                && Math.abs(rx) <= STICK_DEADBAND;
    }

    /** Returns true when hold owns the motors, including zero power while arming/faulted. */
    public boolean update(double y, double x, double rx, boolean bypass, double now) {
        if (bypass || !sticksIdle(y, x, rx)) {
            release();
            status = bypass ? "Bypassed" : "Driving";
            return false;
        }
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
            if (Double.isNaN(idleSince)) idleSince = now;
            if (now - idleSince < ENGAGE_DELAY_SECONDS) {
                backend.stop();
                status = "Arming";
                return true;
            }
            if (target == null) target = pose;
            double headingError = Math.atan2(Math.sin(target.heading() - pose.heading()),
                    Math.cos(target.heading() - pose.heading()));
            if (target.distance(pose) > MAX_DISPLACEMENT_INCHES
                    || Math.abs(headingError) > MAX_HEADING_ERROR) {
                fail("Displaced too far - move stick to rearm");
                return true;
            }
            backend.hold(target, dt);
            status = "Holding";
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
        faulted = true;
        status = reason;
        backend.stop();
    }

    public void release() {
        if (!Double.isNaN(idleSince) || target != null || faulted) backend.stop();
        target = null;
        idleSince = Double.NaN;
        previousTime = Double.NaN;
        faulted = false;
        status = "Ready";
    }

    public String getStatus() { return status; }

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

    /** Exactly the same localizer, drivetrain and Foresight tuning as autonomous. */
    private static final class PedroBackend implements Backend {
        private final Follower follower;
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
            follower = Constants.create(hardwareMap);
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

        @Override public void hold(Pose target, double dt) {
            // Same operation as Follower HOLD; split out so sensor validation precedes output.
            // true honors the hold-point scaling in the existing ForesightConfig too.
            DrivePowers powers = follower.algorithm().calculateHold(follower.drivetrain,
                    target, follower.localizer.state(), true, dt);
            // L1 bound keeps every mixed wheel <= the hold power limit, preserving ratios.
            follower.drivetrain.drive(limitPowers(powers), false);
        }

        @Override public void stop() {
            follower.stop();
            // Follower.stop() only changes mode; force the actual motor zero immediately.
            follower.drivetrain.stop(true);
            // Manual TeleOp also writes these motors, so Pedro's cache alone cannot zero them.
            for (DcMotor motor : motors) motor.setPower(0);
            follower.algorithm().reset();
        }
    }
}
