package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import java.util.Collections;
import java.util.Map;

/** Run against the pinned Pedro core and FTC SDK, not replacement controller stubs. */
public final class PositionHoldChecks {
    private static int checks;
    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }
    private static final class Drive implements Drivetrain {
        public void drive(DrivePowers p, boolean manual) { }
        public void stop() { }
        public void stop(boolean brake) { }
        public Map<String, Object> debug() { return Collections.emptyMap(); }
        public double maxScaling(DrivePowers a, DrivePowers b) {
            double sum = Math.abs(b.forward()) + Math.abs(b.strafe()) + Math.abs(b.turn());
            return sum <= 1 ? 1 : 1 / sum;
        }
        public double interpolateVelocity(double f, double s, double angle) {
            return 1 / (Math.abs(Math.cos(angle)) / f + Math.abs(Math.sin(angle)) / s);
        }
    }
    private static final class Backend implements TeleOpPositionHold.Backend {
        Pose pose = Pose.zero(), target;
        DrivePowers powers = DrivePowers.zero();
        boolean bad;
        int holds;
        final Foresight controller = new Foresight(Constants.foresightConfig);
        public Pose updatePose() {
            if (bad) throw new IllegalStateException("disconnected");
            return pose;
        }
        public void hold(Pose p, double dt) {
            target = p;
            holds++;
            powers = TeleOpPositionHold.limitPowers(controller.calculateHold(new Drive(), p,
                    MotionState.ofVelocity(pose, Velocity.zero()), true, dt));
        }
        public void stop() { powers = DrivePowers.zero(); controller.reset(); }
    }
    private static TeleOpPositionHold arm(Backend b) {
        TeleOpPositionHold h = new TeleOpPositionHold(b);
        check(h.update(0, 0, 0, false, 0), "idle owns motors during delay");
        check(b.holds == 0, "no correction before delay");
        h.update(0, 0, 0, false, .16);
        check(b.holds == 1, "engages after delay");
        return h;
    }
    private static void zero(Backend b) {
        check(b.powers.forward() == 0 && b.powers.strafe() == 0 && b.powers.turn() == 0,
                "must stop immediately");
    }
    public static void main(String[] args) {
        Backend b = new Backend();
        TeleOpPositionHold h = arm(b);
        b.pose = new Pose(2, 0, 0);
        h.update(0, 0, 0, false, .18);
        check(b.powers.forward() < 0, "opposes forward push with real tuning");
        check(b.target.x() == 0, "target remains fixed after a push");
        b.pose = new Pose(0, 2, 0);
        h.update(0, 0, 0, false, .20);
        check(b.powers.strafe() < 0, "opposes left push");
        b.pose = new Pose(0, 0, .10);
        h.update(0, 0, 0, false, .22);
        check(b.powers.turn() < 0, "opposes CCW rotation");
        check(!h.update(.5, 0, 0, false, .24), "driver wins in same loop");
        zero(b);
        b.pose = new Pose(6, 4, 0);
        h.update(0, 0, 0, false, .26);
        h.update(0, 0, 0, false, .42);
        check(b.target.x() == 6 && b.target.y() == 4, "captures a new stop position");
        check(!h.update(0, 0, 0, true, .44), "alignment/bypass wins");
        zero(b);
        check(TeleOpPositionHold.sticksIdle(.02, -.03, .01), "stick noise deadband");
        check(!TeleOpPositionHold.sticksIdle(0, 0, .07), "rotation releases hold");

        for (double heading : new double[] {Math.PI / 2, Math.PI, -Math.PI / 2}) {
            b = new Backend(); b.pose = new Pose(0, 0, heading); h = arm(b);
            b.pose = new Pose(2 * Math.cos(heading), 2 * Math.sin(heading), heading);
            h.update(0, 0, 0, false, .18);
            check(b.powers.forward() < 0, "field transform opposes push at rotated headings");
        }
        b = new Backend(); b.pose = new Pose(0, 0, Math.toRadians(179)); h = arm(b);
        b.pose = new Pose(0, 0, Math.toRadians(-179));
        h.update(0, 0, 0, false, .18);
        check(h.getStatus().equals("Holding") && b.powers.turn() < 0, "heading wraps shortest way");

        for (int fault = 0; fault < 5; fault++) {
            b = new Backend(); h = arm(b);
            if (fault == 0) b.bad = true;
            if (fault == 1) b.pose = new Pose(Double.NaN, 0, 0);
            if (fault == 2) b.pose = new Pose(13, 0, 0);
            if (fault == 3) b.pose = new Pose(0, 0, Math.PI / 2);
            h.update(0, 0, 0, false, fault == 4 ? .5 : .18);
            zero(b);
            int calls = b.holds;
            b.bad = false; b.pose = Pose.zero();
            h.update(0, 0, 0, false, .52);
            check(b.holds == calls, "fault remains latched after sensor recovery");
            h.update(.5, 0, 0, false, .54);
            h.update(0, 0, 0, false, .56);
            h.update(0, 0, 0, false, .72);
            check(b.holds > calls, "driver can rearm after a fault");
            h.release(); zero(b);
        }
        for (double f : new double[] {-10, -.1, 0, .1, 10}) {
            for (double s : new double[] {-10, 0, 10}) {
                for (double t : new double[] {-10, 0, 10}) {
                    DrivePowers p = TeleOpPositionHold.limitPowers(new DrivePowers(f, s, t));
                    check(Math.abs(p.forward()) + Math.abs(p.strafe()) + Math.abs(p.turn())
                            <= TeleOpPositionHold.MAX_HOLD_POWER + 1e-12, "wheel power bound");
                }
            }
        }
        boolean rejected = false;
        try { TeleOpPositionHold.limitPowers(new DrivePowers(Double.NaN, 0, 0)); }
        catch (IllegalStateException expected) { rejected = true; }
        check(rejected, "reject nonfinite power");
        System.out.println("Position hold: " + checks + " checks passed using real Pedro Constants");
    }
}
