package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Subsystem responsible for automated AprilTag alignment using Limelight vision feedback.
 * Employs PD (Proportional-Derivative) control with exponential smoothing filter for rotation and strafing.
 */
public class AprilTagAlignment {

    // --------------------- Hardware Dependencies ---------------------
    private Limelight limelight;
    private ElapsedTime runtime;

    // --------------------- Strafe Control Variables ---------------------
    double strafeMultiplier = 1; // 1 = full manual joystick, 0 = automated control
    double addedStrafe = 0;      // Output power injected into MecanumDrive
    double kP_strafe = 1.5;      // Proportional gain for strafe
    double kD_strafe = 0.05;     // Derivative gain for strafe
    double strafeDerivative = 0;
    double rawStrafeError = 0;
    double lastStrafeError = 0;
    double strafeOutput = 0;
    private static final double MAX_STRAFE_OUTPUT = 1.0;
    double strafeTolerance = 0.02; // Deadband tolerance (~2cm)

    // --------------------- Rotation Control Variables ---------------------
    double rotationMultiplier = 1; // 1 = full manual joystick, 0 = automated control
    double addedRotation = 0;      // Output power injected into MecanumDrive
    double kP_rotation = 0.02;     // Proportional gain for rotation
    double kD_rotation = 0.0;      // Derivative gain for rotation
    double rotationDerivative = 0;
    double rawRotError = 0;
    double lastRotError = 0;
    double rotOutput = 0;
    private static final double MAX_ROTATION_OUTPUT = 1.0;
    double angleTolerance = 0.2;  // Deadband tolerance (degrees)

    // --------------------- Shared State Variables ---------------------
    double dt = 0;
    double curTime = 0;
    double lastTime = 0;
    private boolean targetWasVisible = false;
    double goalX = 0; // Desired target offset (centered)

    double smoothedRotError = 0;
    double smoothedStrafeError = 0;

    // Low-pass exponential moving average filter constant to smooth out noisy vision readings
    private static final double FILTER_GAIN = 0.7;

    // ----------------- Incremental PID Tuning Variables -----------------
    double[] stepSizes = {1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 2;
    int modifiableIndex = 0;
    double[] modifiableValues = {
            kP_rotation,
            kD_rotation,
            /*
            kP_strafe,
            kD_strafe
            */
    };

    /**
     * Initializes the AprilTagAlignment subsystem with default PID constants.
     */
    public void init(Limelight limelight, ElapsedTime runtime) {
        this.limelight = limelight;
        this.runtime = runtime;
    }

    /**
     * Overloaded initialization with custom rotation PID constants.
     */
    public void init(Limelight limelight, ElapsedTime runtime, double kPValueRot, double kDValueRot) {
        this.limelight = limelight;
        this.runtime = runtime;
        this.kP_rotation = kPValueRot;
        this.kD_rotation = kDValueRot;
    }

    /**
     * Overloaded initialization with custom rotation and strafe PID constants.
     */
    public void init(Limelight limelight, ElapsedTime runtime, double kPValueRot, double kDValueRot, double kPValueStrafe, double kDValueStrafe) {
        this.limelight = limelight;
        this.runtime = runtime;
        this.kP_rotation = kPValueRot;
        this.kD_rotation = kDValueRot;
        this.kP_strafe = kPValueStrafe;
        this.kD_strafe = kDValueStrafe;
    }

    /**
     * Resets runtime timers and clears error history for a fresh start.
     */
    public void start() {
        runtime.reset();
        curTime = runtime.time();
        lastTime = curTime;

        rawRotError = 0;
        lastRotError = 0;
        rotationDerivative = 0;
        rotOutput = 0;

        rawStrafeError = 0;
        lastStrafeError = 0;
        strafeDerivative = 0;
        strafeOutput = 0;

        targetWasVisible = false;
    }

    /**
     * Main subsystem update loop called on every frame.
     */
    public void update() {
        handlePDupdates();
    }

    /**
     * Updates the PD calculations using Limelight target data.
     * Applies exponential low-pass filtering and calculates error derivatives.
     */
    private void handlePDupdates() {
        lastTime = curTime;
        curTime = runtime.time();
        dt = curTime - lastTime;

        boolean targetVisible = limelight.isTargetVisible();

        if (targetVisible) {
            double newRotError = goalX - limelight.getHorizontalDelta();
            double newStrafeError = goalX - limelight.get3DXDistance();

            // First frame target acquired: initialize error history without derivative spike
            if (!targetWasVisible) {
                smoothedRotError = newRotError;
                smoothedStrafeError = newStrafeError;

                rawRotError = newRotError;
                lastRotError = newRotError;
                rotationDerivative = 0;

                rawStrafeError = newStrafeError;
                lastStrafeError = newStrafeError;
                strafeDerivative = 0;
            } else {
                // Exponential low-pass filter to smooth vision measurement noise
                smoothedRotError = (FILTER_GAIN * newRotError) + ((1.0 - FILTER_GAIN) * smoothedRotError);
                smoothedStrafeError = (FILTER_GAIN * newStrafeError) + ((1.0 - FILTER_GAIN) * smoothedStrafeError);

                lastRotError = rawRotError;
                rawRotError = smoothedRotError;

                lastStrafeError = rawStrafeError;
                rawStrafeError = smoothedStrafeError;

                // Calculate error derivative over change in time (dt)
                if (dt > 0.000001) {
                    rotationDerivative = (rawRotError - lastRotError) / dt;
                    strafeDerivative = (rawStrafeError - lastStrafeError) / dt;
                } else {
                    rotationDerivative = 0;
                    strafeDerivative = 0;
                }
            }

            // PD Output: Proportional + Derivative
            rotOutput = (kP_rotation * rawRotError) + (kD_rotation * rotationDerivative);
            strafeOutput = (kP_strafe * rawStrafeError) + (kD_strafe * strafeDerivative);

            // Clamp power output to safe maximums
            rotOutput = clamp(rotOutput, -MAX_ROTATION_OUTPUT, MAX_ROTATION_OUTPUT);
            strafeOutput = clamp(strafeOutput, -MAX_STRAFE_OUTPUT, MAX_STRAFE_OUTPUT);
        } else {
            // Target lost: clear error accumulation and outputs
            rawRotError = 0;
            lastRotError = 0;
            rotationDerivative = 0;
            rotOutput = 0;

            rawStrafeError = 0;
            lastStrafeError = 0;
            strafeDerivative = 0;
            strafeOutput = 0;

            smoothedRotError = 0;
            smoothedStrafeError = 0;
        }

        targetWasVisible = targetVisible;
    }

    /**
     * Enables automated rotation toward the target tag if visible.
     * Disables manual joystick rotation (`rotationMultiplier = 0`) and applies calculated PD power.
     */
    public void autoRotate() {
        if (limelight.isTargetVisible()) {
            rotationMultiplier = 0; // Lock manual joystick rotation
            if (Math.abs(rawRotError) < angleTolerance) {
                addedRotation = 0; // Within angle tolerance deadband
            } else {
                addedRotation = -rotOutput;
            }
        } else {
            setManualRotation();
        }
    }

    /**
     * Disables automated rotation alignment and restores manual driver rotation control.
     */
    public void setManualRotation() {
        rotationMultiplier = 1; // Restore manual joystick rotation
        addedRotation = 0;
    }

    /**
     * Enables automated strafing toward the target tag if visible.
     * Disables manual joystick strafing (`strafeMultiplier = 0`) and applies calculated PD power.
     */
    public void autoStrafe() {
        if (limelight.isTargetVisible()) {
            strafeMultiplier = 0; // Lock manual joystick strafe
            if (Math.abs(rawStrafeError) < strafeTolerance) {
                addedStrafe = 0; // Within distance tolerance deadband
            } else {
                addedStrafe = strafeOutput;
            }
        } else {
            setManualStrafing();
        }
    }

    /**
     * Disables automated strafe alignment and restores manual driver strafe control.
     */
    public void setManualStrafing() {
        strafeMultiplier = 1; // Restore manual joystick strafe
        addedStrafe = 0;
    }

    /**
     * Increases the currently selected PID parameter by the active step size.
     */
    public void addIndexValue() {
        modifiableValues[modifiableIndex] += stepSizes[stepIndex];
        syncModifiableValues();
    }

    /**
     * Decreases the currently selected PID parameter by the active step size.
     */
    public void subIndexValue() {
        modifiableValues[modifiableIndex] -= stepSizes[stepIndex];
        syncModifiableValues();
    }

    /**
     * Cycles through modifiable PID parameters (kP_rotation, kD_rotation).
     */
    public void switchPIDModifyingValue() {
        modifiableIndex = (modifiableIndex + 1) % modifiableValues.length;
    }

    /**
     * Cycles through tuning step sizes (1.0, 0.1, 0.01, 0.001, 0.0001).
     */
    public void switchStepSize() {
        stepIndex = (stepIndex + 1) % stepSizes.length;
    }

    /**
     * Syncs modified values array back to individual PID fields.
     */
    private void syncModifiableValues() {
        kP_rotation = modifiableValues[0];
        kD_rotation = modifiableValues[1];
        /*
        kP_strafe = modifiableValues[2];
        kD_strafe = modifiableValues[3];
        */
    }

    /**
     * Helper utility to clamp a value between min and max bounds.
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // --------------------- Getters ---------------------
    public double getRotationMultiplier() {
        return rotationMultiplier;
    }
    public double getAddedRotation(){return addedRotation;}
    public double getStrafeMultiplier(){return strafeMultiplier;}
    public double getAddedStrafe(){return addedStrafe;}
    public double getkP_strafe(){return kP_strafe;}
    public double getkD_strafe(){return kD_strafe;}
    public double getStepSize() {
        return stepSizes[stepIndex];
    }
    public double getkP_rotation() {
        return kP_rotation;
    }
    public double getkD_rotation() {
        return kD_rotation;
    }
    public String getCurrentlyModifying() {
        String[] names = {
                "kP_rotation",
                "kD_rotation",
                /*
                "kP_strafe",
                "kD_strafe"
                */
        };
        return names[modifiableIndex];
    }
}
