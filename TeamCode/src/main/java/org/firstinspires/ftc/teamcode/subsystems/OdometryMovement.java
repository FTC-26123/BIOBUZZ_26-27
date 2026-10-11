package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.odometry.GoBildaPinpointDriver;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Subsystem responsible for closed-loop distance movements using the goBILDA Pinpoint Odometry Computer.
 * Integrates directly with MecanumDrive using multiplier locks and power offsets.
 */
public class OdometryMovement {

    private static final Logger log = LoggerFactory.getLogger(OdometryMovement.class);
    private GoBildaPinpointDriver pinpoint;
    private MecanumDrive driveTrain;

    // --------------------- Input & Target Variables ---------------------
    double inputDistance = 0;
    int[] increments = {1, 2, 6, 12};
    int incrementIndex = 0;
    String[] direction = {"forward", "strafe"};
    int directionIndex = 0;

    double error = 0; // Current distance error (target - traveled)

    // --------------------- Drivetrain Multipliers & Offsets ---------------------
    double strafeMultiplier = 1;  // 1 = manual joystick enabled, 0 = locked for auto-strafe
    double addedStrafe = 0;       // Output strafe power sent to MecanumDrive
    double forwardMultiplier = 1; // 1 = manual joystick enabled, 0 = locked for auto-forward
    double addedForward = 0;      // Output forward power sent to MecanumDrive

    double forwardOutput = 0;
    double strafeOutput = 0;

    public static final String TAG = "Odometry";

    // --------------------- Distance Tracking State ---------------------
    private double targetDistance = 0;
    private double startYInches = 0;
    private double startXInches = 0;
    private boolean isDrivingY = false;
    private boolean isStrafingX = false;
    private static final double DISTANCE_TOLERANCE_INCHES = 0.5; // Arrived deadband tolerance (inches)

    /**
     * Cycles through distance step size increments (1, 2, 6, 12 inches).
     */
    public void cycleIncrement() {
        incrementIndex = (incrementIndex + 1) % increments.length;
    }

    public int getSelectedIncrement() {
        return increments[incrementIndex];
    }

    /**
     * Cycles through travel direction ("forward", "strafe").
     */
    public void cycleDirection() {
        directionIndex = (directionIndex + 1) % direction.length;
    }

    public String getSelectedDirection() {
        return direction[directionIndex];
    }

    public double getInputDistance() {
        return inputDistance;
    }

    /**
     * @return True if the robot is currently executing an automated distance drive movement.
     */
    public boolean isBusy() {
        return isDrivingY || isStrafingX;
    }

    public double getError() {
        return error;
    }

    public double getTargetDistance() {
        return targetDistance;
    }

    /**
     * @return Status description for Driver Station telemetry.
     */
    public String getStatus() {
        if (isDrivingY) return String.format("Driving Forward (Target: %.1f in)", targetDistance);
        if (isStrafingX) return String.format("Strafing (Target: %.1f in)", targetDistance);
        return "Idle";
    }

    /**
     * Initializes the goBILDA Pinpoint Odometry Computer hardware, pod offsets, and directions.
     */
    public void init(HardwareMap hwMap, MecanumDrive driveTrain) {
        this.driveTrain = driveTrain;
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(2.0, -7.0); // Pod offsets in mm relative to tracking center
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
    }

    public double getStartXInches() {
        return startXInches;
    }

    public double getStartYInches() {
        return startYInches;
    }

    /**
     * Main update loop called in TeleOp/Auton frame loops.
     */
    public void update(Gamepad gamepad1) {
        pinpoint.update();
        handleManualDistanceInput(gamepad1);
        updateDistanceMovement();
        Log.d(TAG, "X" + getHorizantalValue() + "Error" + getError() + "Target" + getTargetDistance() + "Status" + getStatus() + "Start X" + getStartXInches() + "StrafeMultiplier" + getStrafeMultiplier() + "addedStrafe" + getAddedStrafe() + "ForwardMultiplier" + getForwardMultiplier() + "AddedForward" + getAddedForward());
    }

    private Pose2D getPose() {
        pinpoint.update();
        return pinpoint.getPosition();
    }

    /**
     * @return Horizontal position in inches (mapped from Pinpoint Y position).
     */
    public double getHorizantalValue() {
        return getPose().getY(DistanceUnit.INCH);
    }

    /**
     * @return Vertical position in inches (mapped from Pinpoint X position).
     */
    public double getVerticalValue() {
        return getPose().getX(DistanceUnit.INCH);
    }

    /**
     * @return Heading orientation in degrees.
     */
    public double getHeadingDegrees() {
        return getPose().getHeading(AngleUnit.DEGREES);
    }

    /**
     * Initiates automated distance movement in the requested direction.
     */
    public void startDistanceMovement(String direction, double distance) {
        targetDistance = distance;
        inputDistance = 0; // Reset input distance at start

        if (direction.equalsIgnoreCase("forward")) {
            startYInches = getVerticalValue();
            isDrivingY = true;
            isStrafingX = false;
        } else if (direction.equalsIgnoreCase("strafe")) {
            startXInches = getHorizantalValue();
            isStrafingX = true;
            isDrivingY = false;
        }
    }

    /**
     * Closed-loop distance controller executed inside update().
     * Calculates distance error, applies proportional power, clamps speed, and stops at deadband tolerance.
     */
    private void updateDistanceMovement() {
        if (isDrivingY) {
            double distanceTraveled = getVerticalValue() - startYInches;
            error = targetDistance - distanceTraveled;

            if (Math.abs(error) <= DISTANCE_TOLERANCE_INCHES) {
                stopMoving();
            } else {
                double kP = 0.2;
                forwardMultiplier = 0; // Lock manual joystick y-axis
                addedForward = -(kP * error); // Negative power due to MecanumDrive -y equation

                // Clamp maximum output magnitude to 0.8
                if ((Math.min(Math.abs(addedForward), 0.8)) != Math.abs(addedForward)) {
                    if (error < 0) {
                        addedForward = 0.8;
                    } else {
                        addedForward = -0.8;
                    }
                }
                // Enforce minimum output power floor of 0.20 to prevent stalling
                if ((Math.max(Math.abs(addedForward), 0.20)) != Math.abs(addedForward)) {
                    if (error < 0) {
                        addedForward = 0.20;
                    } else {
                        addedForward = -0.20;
                    }
                }
                strafeMultiplier = 0;
                addedStrafe = 0;
            }
        } else if (isStrafingX) {
            double distanceTraveled = getHorizantalValue() - startXInches;
            error = targetDistance - distanceTraveled;

            if (Math.abs(error) <= DISTANCE_TOLERANCE_INCHES) {
                stopMoving();
            } else {
                double kP = 0.2;
                strafeMultiplier = 0; // Lock manual joystick x-axis
                addedStrafe = kP * error;

                // Clamp maximum output magnitude to 0.8
                if ((Math.min(Math.abs(addedStrafe), 0.8)) != Math.abs(addedStrafe)) {
                    if (error < 0) {
                        addedStrafe = -0.8;
                    } else {
                        addedStrafe = 0.8;
                    }
                }
                // Enforce minimum output power floor of 0.20 to prevent stalling
                if ((Math.max(Math.abs(addedStrafe), 0.20)) != Math.abs(addedStrafe)) {
                    if (error < 0) {
                        addedStrafe = -0.20;
                    } else {
                        addedStrafe = 0.20;
                    }
                }
                forwardMultiplier = 0;
                addedForward = 0;
            }
        } else {
            // Idle state: restore manual driver joysticks
            strafeMultiplier = 1;
            addedStrafe = 0;
            forwardMultiplier = 1;
            addedForward = 0;
        }
    }

    /**
     * Convenience helper to drive forward by specified inches.
     */
    public void driveForwardInches(double distanceInches) {
        startDistanceMovement("forward", distanceInches);
    }

    /**
     * Convenience helper to strafe by specified inches.
     */
    public void strafeInches(double distanceInches) {
        startDistanceMovement("strafe", distanceInches);
    }

    /**
     * Immediately stops active movement and restores full manual driver joystick control.
     */
    public void stopMoving() {
        isDrivingY = false;
        isStrafingX = false;
        targetDistance = 0;
        error = 0;
        if (driveTrain != null) {
            driveTrain.drive(0, 0, 0);
        }
        forwardMultiplier = 1;
        strafeMultiplier = 1;
        addedStrafe = 0;
        addedForward = 0;
    }

    /**
     * Process Gamepad 1 button inputs for testing distance movement.
     */
    private void handleManualDistanceInput(Gamepad gamepad1) {
        if (gamepad1.aWasPressed()) {
            cycleDirection();
        }
        if (gamepad1.bWasPressed()) {
            cycleIncrement();
        }
        if (gamepad1.dpadUpWasPressed()) {
            inputDistance += getSelectedIncrement();
        } else if (gamepad1.dpadDownWasPressed()) {
            inputDistance -= getSelectedIncrement();
        }

        // Start movement on Right Trigger
        if (gamepad1.right_trigger > 0.3 && !isBusy() && inputDistance != 0) {
            startDistanceMovement(getSelectedDirection(), inputDistance);
        }

        // Emergency stop on Right Bumper
        if (gamepad1.right_bumper) {
            stopMoving();
        }
    }

    public void handleDistanceDriving(String direction, double distance) {
        startDistanceMovement(direction, distance);
    }

    // --------------------- Getters ---------------------
    public double getStrafeMultiplier() {
        return strafeMultiplier;
    }

    public double getAddedStrafe() {
        return addedStrafe;
    }

    public double getForwardMultiplier() {
        return forwardMultiplier;
    }

    public double getAddedForward() {
        return addedForward;
    }
}
