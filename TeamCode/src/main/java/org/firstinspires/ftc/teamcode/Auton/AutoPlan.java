package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Bounded autonomous stages; a failed stage skips the remaining scoring attempts. */
public final class AutoPlan {
    public static final double RUN_TIMEOUT_SECONDS = 27.0;
    public static final double PARK_RESERVE_SECONDS = 5.0;
    public static final double NO_PROGRESS_SECONDS = 2.0;
    private final Follower follower;
    private final BooleanSupplier localizationReady;
    private final List<Stage> stages = new ArrayList<>();
    private final ElapsedTime runTimer = new ElapsedTime();
    private final ElapsedTime stageTimer = new ElapsedTime();
    private final ElapsedTime progressTimer = new ElapsedTime();
    private Stage park;
    private Stage activeStage;
    private Command active;
    private Pose progressPose;
    private int index;
    private boolean parking;
    private boolean finished;
    private String failure = "";
    private String result = "Idle";

    private static final class Stage {
        final String name;
        final Supplier<Command> factory;
        final double timeout;
        final boolean motion;
        final BooleanSupplier succeeded;
        Stage(String name, Supplier<Command> factory, double timeout,
              boolean motion, BooleanSupplier succeeded) {
            this.name = name;
            this.factory = factory;
            this.timeout = timeout;
            this.motion = motion;
            this.succeeded = succeeded;
        }
    }

    public AutoPlan(Follower follower, BooleanSupplier localizationReady) {
        this.follower = follower;
        this.localizationReady = localizationReady;
    }

    /** Wait for Pedro's complete endpoint criteria, not just the curve's parametric end. */
    public static Command follow(Follower follower, Supplier<Path> path) {
        return Command.build().setStart(() -> follower.follow(path.get()))
                .setDone(() -> !follower.isBusy()).setEnd(condition -> {
                    if (condition != EndCondition.NATURALLY) {
                        follower.stop();
                        follower.drivetrain.stop();
                    }
                });
    }

    public AutoPlan add(String name, Supplier<Command> factory, double timeout,
                        boolean motion, BooleanSupplier succeeded) {
        stages.add(new Stage(name, factory, timeout, motion, succeeded));
        return this;
    }

    public AutoPlan park(Supplier<Command> factory) {
        park = new Stage("Park", factory, PARK_RESERVE_SECONDS, true, () -> true);
        return this;
    }

    public Command command() {
        return Command.build().setStart(() -> {
            endActive(EndCondition.INTERRUPTED);
            activeStage = null;
            index = 0;
            parking = false;
            finished = false;
            failure = "";
            runTimer.reset();
            if (!usablePose()) fail("Localization unavailable", false);
            else next();
        }).setExecute(this::update).setDone(() -> finished).setEnd(condition -> {
            endActive(EndCondition.INTERRUPTED);
            stopDrive();
            if (!finished) result = "Cancelled";
            finished = true;
        });
    }

    private void next() {
        if (index < stages.size()) begin(stages.get(index++));
        else beginPark();
    }

    private void begin(Stage stage) {
        activeStage = stage;
        active = stage.factory.get();
        stageTimer.reset();
        progressTimer.reset();
        progressPose = follower.pose();
        result = "Running";
        active.start();
    }

    private void beginPark() {
        if (parking || park == null || !usablePose()) {
            finish("Stopped");
            return;
        }
        parking = true;
        begin(park);
    }

    private boolean usablePose() {
        Pose pose = follower.pose();
        return localizationReady.getAsBoolean() && pose != null
                && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private void update() {
        if (finished) return;
        if (!usablePose()) { fail("Localization unavailable", false); return; }
        if (runTimer.seconds() >= RUN_TIMEOUT_SECONDS) {
            fail("Routine deadline", false);
            return;
        }
        if (!parking && runTimer.seconds() >= RUN_TIMEOUT_SECONDS - PARK_RESERVE_SECONDS) {
            fail("Parking reserve reached", true);
            return;
        }
        Pose pose = follower.pose();
        double headingDelta = Math.atan2(Math.sin(pose.heading() - progressPose.heading()),
                Math.cos(pose.heading() - progressPose.heading()));
        if (Math.hypot(pose.x() - progressPose.x(), pose.y() - progressPose.y()) >= 0.25
                || Math.abs(headingDelta) >= Math.toRadians(1)) {
            progressPose = pose;
            progressTimer.reset();
        }
        active.execute();
        if (active.done()) {
            boolean succeeded = activeStage.succeeded.getAsBoolean();
            endActive(EndCondition.NATURALLY);
            if (!succeeded) fail("Stage did not succeed", true);
            else if (parking) finish(failure.isEmpty() ? "Parked" : "Fallback parked");
            else next();
            return;
        }
        // No useful movement could mean an obstruction or a frozen sensor: stop, do not blindly park.
        if (activeStage.motion && progressTimer.seconds() >= NO_PROGRESS_SECONDS) {
            fail("No movement progress", false);
        } else if (stageTimer.seconds() >= activeStage.timeout) {
            fail("Stage deadline", true);
        }
    }

    private void fail(String reason, boolean mayPark) {
        if (failure.isEmpty()) failure = (activeStage == null ? "Init" : activeStage.name) + ": " + reason;
        endActive(EndCondition.INTERRUPTED);
        stopDrive();
        if (mayPark && !parking && usablePose() && runTimer.seconds() < RUN_TIMEOUT_SECONDS) beginPark();
        else finish("Stopped: " + reason);
    }

    private void endActive(EndCondition condition) {
        if (active != null) {
            Command ending = active;
            active = null;
            ending.end(condition);
        }
    }

    private void finish(String outcome) {
        finished = true;
        result = outcome;
        stopDrive();
    }

    private void stopDrive() {
        follower.stop();
        follower.drivetrain.stop();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Auto stage", activeStage == null ? "Init" : activeStage.name);
        telemetry.addData("Stage elapsed (s)", stageTimer.seconds());
        telemetry.addData("Auto result", result);
        telemetry.addData("Auto failure", failure.isEmpty() ? "None" : failure);
    }
}
