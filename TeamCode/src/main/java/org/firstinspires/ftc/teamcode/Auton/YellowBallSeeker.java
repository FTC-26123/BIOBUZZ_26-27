package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import java.util.List;

/** Shares one validated color observation between movement, completion, and telemetry. */
public final class YellowBallSeeker {
    public static final int PIPELINE = 0;
    private static final double ALIGNMENT_DEG = 4;
    private static final double CLOSE_BOTTOM_PX = 380;
    private static final double ACQUISITION_DWELL = 0.15;
    private final Follower follower;
    private final Limelight3A camera;
    private final ElapsedTime searchTimer = new ElapsedTime();
    private final ElapsedTime arrivalTimer = new ElapsedTime();
    private Observation observation;
    private double sampledTimestamp = Double.NaN;
    private double processedTimestamp = Double.NaN;
    private boolean freshFrame;
    private boolean arrivalStarted;
    private boolean acquired;
    private String status = "Idle";

    static final class Observation {
        final double timestamp, tx, area, bottom, height;
        Observation(double timestamp, double tx, double area, double bottom, double height) {
            this.timestamp = timestamp;
            this.tx = tx;
            this.area = area;
            this.bottom = bottom;
            this.height = height;
        }
        boolean atIntake() { return bottom >= CLOSE_BOTTOM_PX && Math.abs(tx) <= ALIGNMENT_DEG; }
    }

    public YellowBallSeeker(Follower follower, Limelight3A camera) {
        this.follower = follower;
        this.camera = camera;
    }

    /** Call exactly once per OpMode loop before executing the scheduler. */
    public void sample() {
        LLResult result = camera.getLatestResult();
        freshFrame = Limelight.isFresh(result) && result.getPipelineIndex() == PIPELINE
                && Double.isFinite(result.getTimestamp());
        if (!freshFrame) {
            observation = null;
            sampledTimestamp = Double.NaN;
        } else if (result.getTimestamp() != sampledTimestamp) {
            sampledTimestamp = result.getTimestamp();
            observation = select(result);
        }
    }

    static Observation select(LLResult result) {
        List<LLResultTypes.ColorResult> targets = result.getColorResults();
        if (targets == null) return null;
        Observation best = null;
        for (LLResultTypes.ColorResult target : targets) {
            if (target == null) continue;
            double tx = target.getTargetXDegrees();
            double area = target.getTargetArea();
            if (!Double.isFinite(tx) || !Double.isFinite(area) || area <= 0) continue;
            List<List<Double>> corners = target.getTargetCorners();
            if (corners == null) continue;
            double top = Double.POSITIVE_INFINITY, bottom = Double.NEGATIVE_INFINITY;
            for (List<Double> corner : corners) {
                if (corner == null || corner.size() < 2 || corner.get(1) == null) continue;
                double y = corner.get(1);
                if (!Double.isFinite(y)) continue;
                top = Math.min(top, y);
                bottom = Math.max(bottom, y);
            }
            // Existing camera calibration: 12-inch camera, image center y=240.
            // These are preserved setup assumptions, not newly measured geometry.
            if (!Double.isFinite(top) || !Double.isFinite(bottom) || bottom <= top || bottom - 240 <= 12) continue;
            double height = 12 * (bottom - top) / (bottom - 240);
            if (!Double.isFinite(height) || Math.abs(height - 2.8) > 0.7) continue;
            if (best == null || area > best.area) best = new Observation(result.getTimestamp(), tx, area, bottom, height);
        }
        return best;
    }

    public Command command() {
        return Command.build().setStart(() -> {
            searchTimer.reset();
            acquired = false;
            arrivalStarted = false;
            processedTimestamp = Double.NaN;
            sampledTimestamp = Double.NaN;
            observation = null;
            freshFrame = false;
            camera.pipelineSwitch(PIPELINE);
            status = "Searching";
            follower.manual(0, 0, 0);
        }).setExecute(() -> {
            if (!freshFrame) {
                arrivalStarted = false;
                status = "Waiting for fresh color frames";
                follower.manual(0, 0, 0);
                return;
            }
            if (observation == null) {
                arrivalStarted = false;
                status = "Searching";
                follower.manual(0, 0, 0.15);
                return;
            }
            if (observation.timestamp == processedTimestamp) return;
            processedTimestamp = observation.timestamp;
            double turn = Math.max(-0.35, Math.min(0.35, 0.025 * observation.tx));
            if (observation.atIntake()) {
                follower.manual(0, 0, 0);
                if (!arrivalStarted) { arrivalStarted = true; arrivalTimer.reset(); }
                acquired = arrivalTimer.seconds() >= ACQUISITION_DWELL;
                status = acquired ? "Aligned visual arrival" : "Confirming visual arrival";
            } else {
                arrivalStarted = false;
                double forward = observation.bottom >= CLOSE_BOTTOM_PX || Math.abs(observation.tx) > 10
                        ? 0 : Math.abs(observation.tx) > ALIGNMENT_DEG ? 0.25 : 0.55;
                status = "Approaching";
                follower.manual(forward, 0, turn);
            }
        }).setDone(() -> acquired || searchTimer.seconds() >= 6).setEnd(condition -> {
            follower.manual(0, 0, 0);
            follower.drivetrain.stop();
            if (!acquired) status = "Search ended without visual arrival";
        });
    }

    public boolean acquired() { return acquired; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Yellow seek status", status);
        if (observation != null) {
            telemetry.addData("Yellow TX", observation.tx);
            telemetry.addData("Yellow area", observation.area);
            telemetry.addData("Yellow bottom Y", observation.bottom);
            telemetry.addData("Estimated height (in)", observation.height);
        }
    }
}
