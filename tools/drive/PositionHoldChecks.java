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
        // Nothing moves the robot right after Play: the hold waits until the driver has driven once.
        check(!h.update(0, 0, 0, false, -.2), "no hold before the first drive");
        check(b.holds == 0, "no correction before the first drive");
        check(!h.update(.5, 0, 0, false, -.1), "driver drives");
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

        // A sustained shove from another robot (50 in/s, well past the old 12-inch cut-off) is resisted throughout.
        b = new Backend(); h = arm(b);
        double clock = .16;
        for (int step = 1; step <= 30; step++) {
            clock += .02; b.pose = new Pose(step, 0, 0);
            check(h.update(0, 0, 0, false, clock), "hold keeps the motors while being pushed");
            check(b.powers.forward() < 0, "keeps pushing back at " + step + " inches");
            check(b.target.x() == 0, "never gives up the original target while pushed");
        }
        check(h.getStatus().equals("Resisting push"), "telemetry reports an active push");
        // Being spun by contact is resisted too (up to MAX_HOLD_HEADING, 30 degrees).
        b = new Backend(); h = arm(b); clock = .16;
        for (int step = 1; step <= 7; step++) {
            clock += .02; b.pose = new Pose(0, 0, Math.toRadians(4 * step));
            h.update(0, 0, 0, false, clock);
            check(b.powers.turn() < 0, "keeps turning back at " + 4 * step + " degrees");
        }
        // RUNAWAY SPIN: the robot keeps turning although the hold pushes back (heading direction reversed).
        // It must stop turning by 30 degrees, and after the second runaway switch the hold off for good.
        b = new Backend(); h = arm(b); clock = .16;
        for (int step = 1; step <= 12; step++) { clock += .02; b.pose = new Pose(0, 0, Math.toRadians(4 * step)); h.update(0, 0, 0, false, clock); }
        zero(b);
        check(h.getStatus().startsWith("Turned"), "runaway spin stopped within 30 degrees: " + h.getStatus());
        h.update(.5, 0, 0, false, clock += .02);                 // driver re-arms
        b.pose = new Pose(0, 0, Math.toRadians(48));
        h.update(0, 0, 0, false, clock += .02);
        h.update(0, 0, 0, false, clock += .2);                    // engaged again at 48 deg
        for (int step = 1; step <= 12; step++) { clock += .02; b.pose = new Pose(0, 0, Math.toRadians(48 + 4 * step)); h.update(0, 0, 0, false, clock); }
        zero(b);
        check(h.getStatus().startsWith("OFF"), "second runaway switches the hold off: " + h.getStatus());
        int holdsWhenOff = b.holds;
        h.update(.5, 0, 0, false, clock += .02);
        check(!h.update(0, 0, 0, false, clock += .5) && b.holds == holdsWhenOff, "hold stays off for the rest of the OpMode");
        // Carried past the hold range: stop rather than drive back across the field.
        b = new Backend(); h = arm(b); clock = .16;
        for (int step = 1; step <= 40; step++) { clock += .02; b.pose = new Pose(step, 0, 0); h.update(0, 0, 0, false, clock); }
        zero(b);
        check(h.getStatus().startsWith("Pushed beyond hold range"), "carry past range stops the hold");
        // An odometry jump far faster than any push is a fault, not a target to chase.
        b = new Backend(); h = arm(b);
        b.pose = new Pose(5, 0, 0); h.update(0, 0, 0, false, .18);
        zero(b);
        check(h.getStatus().startsWith("Odometry jump"), "pose jump is treated as an odometry fault");

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
