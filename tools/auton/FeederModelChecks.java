import org.firstinspires.ftc.teamcode.Auton.cycles.FeederModel;
import org.firstinspires.ftc.teamcode.Auton.cycles.FlywheelForecast;

/** Standalone JVM regression checks; no simulator or Android hardware stubs. */
public final class FeederModelChecks {
    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }
    public static void main(String[] args) {
        FeederModel positive = new FeederModel(), negative = new FeederModel();
        positive.observe(0, 1800); negative.observe(0, -1800);
        positive.begin(0); negative.begin(0);
        double initial = positive.remaining(0, 1800, -10);
        check(Double.isFinite(initial) && initial > 0, "Finite initial transport estimate");
        for (int i = 1; i <= 10; i++) {
            double now = i * .01;
            positive.observe(now, 1800); negative.observe(now, -1800);
            check(Math.abs(positive.progressTicks() - negative.progressTicks()) < 1e-9,
                    "Real intake polarity must not change estimated travel");
        }
        check(positive.remaining(.1, 1800, -10) < initial, "Transport reduces remaining time");
        positive.pause(); double progress = positive.progressTicks();
        positive.observe(.2, 1800);
        check(positive.progressTicks() == progress, "Closed gate freezes transport");
        positive.cancel();
        check(!positive.active() && positive.progressTicks() == 0, "Cancel clears partial feed");
        positive.begin(.3);
        check(positive.active() && positive.progressTicks() == 0, "Restart discards cancelled travel");
        check(Double.isNaN(positive.remaining(Double.NaN, 1800, -10)), "Reject invalid clock");
        check(Double.isNaN(FlywheelForecast.after(Double.NaN, 1000, .1)), "Reject invalid speed");
        double recovery = FlywheelForecast.after(0, 1800, .1);
        check(recovery > 0 && recovery < 1800, "Recovery requires elapsed time");
        check(FlywheelForecast.after(recovery, 1800, .1) > recovery, "Recovery progresses");
        check(FlywheelForecast.after(1200, 0, .1) < 1200, "Stop command decelerates wheel");
        double command = FlywheelForecast.command(0, 2200, .01);
        check(command >= 0 && command <= 2240, "Feed compensation respects motor limit");
        System.out.println("Feeder and flywheel checks passed");
    }
}
