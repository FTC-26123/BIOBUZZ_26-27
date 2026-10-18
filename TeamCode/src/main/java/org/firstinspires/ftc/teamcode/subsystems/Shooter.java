package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Flywheel control with explicit firing intent and speed recovery interlock. */
public class Shooter {
    private DcMotorEx shooter;
    private Servo gate;
    private Intake intake;
    private static final double GATE_OPEN = 0.27;
    private static final double GATE_CLOSED = 0.5;
    private static final double VELOCITY_TOLERANCE = 50;
    private static final long READY_DWELL_NANOS = 150_000_000L;
    private final double[] launchingPowers = {2000, 1500, 1000};
    private int launchingIndex;
    private boolean enabled;
    private boolean fireRequested;
    private boolean feeding;
    private boolean ready;
    private long readySince = -1;
    private long intakeRevision;
    private String shooterStatus = "Off";
    private String gateStatus = "Closed";

    public void init(HardwareMap hwMap, Intake intake) {
        this.intake = intake;
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        gate = hwMap.tryGet(Servo.class, "gate");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        stop();
    }

    public void start() {
        if (!enabled) resetReadiness();
        enabled = true;
        shooter.setVelocity(getTargetLaunchPower());
    }

    /** Hold a firing control to feed; starting the flywheel alone never feeds. */
    public void setFireRequested(boolean requested) {
        fireRequested = requested && enabled;
    }

    public void stop() {
        enabled = false;
        fireRequested = false;
        feeding = false;
        resetReadiness();
        shooterStatus = "Off";
        shooter.setVelocity(0);
        intake.stop();
        intakeRevision = intake.getCommandRevision();
        setGate(false);
    }

    public void switchSpeed() {
        launchingIndex = (launchingIndex + 1) % launchingPowers.length;
        resetReadiness();
        if (enabled) shooter.setVelocity(getTargetLaunchPower());
    }

    /** Explicit service/manual override; automatic feeding still requires readiness. */
    public void openGate() { setGate(true); }
    public void closeGate() {
        fireRequested = false;
        setGate(false);
    }
    private void setGate(boolean open) {
        gateStatus = gate == null ? "Not installed" : (open ? "Open" : "Closed");
        if (gate != null) gate.setPosition(open ? GATE_OPEN : GATE_CLOSED);
    }
    private void resetReadiness() { readySince = -1; ready = false; }
    public double getCurrentSpeed() { return shooter.getVelocity(); }
    public double getTargetLaunchPower() { return launchingPowers[launchingIndex]; }
    public String getShooterStatus() { return shooterStatus; }
    public String getGateStatus() { return gateStatus; }
    public boolean isReady() { return ready; }

    public void update() { update(System.nanoTime()); }

    // Package-visible monotonic clock seam for deterministic regression tests.
    void update(long now) {
        boolean manualIntake = intakeRevision != intake.getCommandRevision();
        intakeRevision = intake.getCommandRevision();
        if (manualIntake) fireRequested = false;
        double velocity = shooter.getVelocity();
        if (!enabled || !Double.isFinite(velocity)
                || Math.abs(velocity - getTargetLaunchPower()) > VELOCITY_TOLERANCE) {
            resetReadiness();
        } else {
            if (readySince < 0) readySince = now;
            ready = now - readySince >= READY_DWELL_NANOS;
        }
        boolean shouldFeed = enabled && ready && fireRequested;
        if (shouldFeed) {
            intake.startAutomatic();
            setGate(true);
        } else if (feeding) {
            // Never overwrite a stop/reverse/custom intake command from this loop.
            if (!manualIntake) intake.stopAutomatic();
            setGate(false);
        }
        feeding = shouldFeed;
        shooterStatus = !enabled ? "Off" : feeding ? "Feeding" : ready ? "Ready" : "Spinning up";
    }
}
