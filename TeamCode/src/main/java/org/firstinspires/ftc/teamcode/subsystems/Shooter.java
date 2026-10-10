package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem controlling the high-speed launcher flywheel motor and servo gate.
 * Interacts with Intake to automatically feed game elements when target flywheel velocity is reached.
 */
public class Shooter {

    // ---------------------- Hardware & Subsystem Dependencies ----------------------
    private DcMotorEx shooter;
    private Servo gate;
    private Intake intake;

    // ---------------------- Gate Servo Positions ----------------------
    double GATE_OPEN = 0.27;
    double GATE_CLOSED = 0.5;

    // ---------------------- Status Variables ----------------------
    String shooterStatus = "Off";
    String gateStatus = "Closed";

    // Predefined launch velocity powers (encoder ticks per second)
    double[] launchingPowers = {2000, 1500, 1000};
    int launchingIndex = 0;

    /**
     * Initializes launcher flywheel motor and gate servo from hardware map.
     */
    public void init(HardwareMap hwMap, Intake intake) {
        this.intake = intake;
        shooter = hwMap.get(DcMotorEx.class, "launcher");
        gate = hwMap.get(Servo.class, "gate");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Stops launcher motor, closes gate servo, and stops intake.
     */
    public void stop() {
        shooterStatus = "Off";
        shooter.setVelocity(0);
        intake.stop();
        closeGate();
    }

    /**
     * Cycles target launch velocity power preset (2000 -> 1500 -> 1000 ticks/sec).
     */
    public void switchSpeed() {
        launchingIndex = (launchingIndex + 1) % launchingPowers.length;
    }

    /**
     * Opens the launcher gate servo to allow game elements to pass into flywheel.
     */
    public void openGate() {
        gateStatus = "Open";
        gate.setPosition(GATE_OPEN);
    }

    /**
     * Closes the launcher gate servo.
     */
    public void closeGate() {
        gateStatus = "Closed";
        gate.setPosition(GATE_CLOSED);
    }

    /**
     * Spools up launcher flywheel motor to active target velocity.
     */
    public void start() {
        shooterStatus = "Starting";
        shooter.setVelocity(launchingPowers[launchingIndex]);
    }

    /**
     * @return Current real-time motor velocity in encoder ticks per second.
     */
    public double getCurrentSpeed() {
        return shooter.getVelocity();
    }

    /**
     * @return Target launch velocity preset in encoder ticks per second.
     */
    public double getTargetLaunchPower() {
        return launchingPowers[launchingIndex];
    }

    public String getShooterStatus() {
        return shooterStatus;
    }

    public String getGateStatus() {
        return gateStatus;
    }

    /**
     * @return True if flywheel velocity is within 50 ticks/sec of target speed.
     */
    public boolean isReady() {
        return shooter.getVelocity() >= (launchingPowers[launchingIndex] - 50);
    }

    /**
     * Updates automated shooting sequence on every loop.
     */
    public void update() {
        handleAutoShooting();
    }

    /**
     * Automatically opens gate and starts intake feeder when flywheel reaches target velocity.
     */
    private void handleAutoShooting() {
        if (launchingPowers[launchingIndex] > 0 && isReady()) {
            shooterStatus = "Shooting";
            intake.start();
            openGate();
        }
    }
}
