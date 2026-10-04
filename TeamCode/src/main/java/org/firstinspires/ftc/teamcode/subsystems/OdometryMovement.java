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

public class OdometryMovement {

    private static final Logger log = LoggerFactory.getLogger(OdometryMovement.class);
    private GoBildaPinpointDriver pinpoint;
    private MecanumDrive driveTrain;

    double inputDistance = 0;
    int[] increments = {1, 2, 6, 12};
    int incrementIndex = 0;
    String[] direction = {"forward", "strafe"};
    int directionIndex = 0;

    double error = 0;


    // --------------------- Forward/Strafe multipliers
    double strafeMultiplier = 1;
    double addedStrafe = 0;
    double forwardMultiplier = 1;
    double addedForward = 0;
    
    double forwardOutput = 0;
    
    double strafeOutput = 0;
    
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

    // ---------- Log
    public static final String TAG = "Odometry";
    
    
    // --------------------- Distance tracking variables
    
    private double targetDistance = 0;
    private double startYInches = 0;
    private double startXInches = 0;
    private boolean isDrivingY = false;
    private boolean isStrafingX = false;
    private static final double DISTANCE_TOLERANCE_INCHES = 0.5;

    public void cycleIncrement() {
        incrementIndex = (incrementIndex + 1) % increments.length;
    }

    public int getSelectedIncrement() {
        return increments[incrementIndex];
    }

    public void cycleDirection() {
        directionIndex = (directionIndex + 1) % direction.length;
    }

    public String getSelectedDirection() {
        return direction[directionIndex];
    }

    public double getInputDistance() {
        return inputDistance;
    }

    public boolean isBusy() {
        return isDrivingY || isStrafingX;
    }

    public double getError(){
        return error;
    }

    public double getTargetDistance() {
        return targetDistance;
    }

    public String getStatus() {
        if (isDrivingY) return String.format("Driving Forward (Target: %.1f in)", targetDistance);
        if (isStrafingX) return String.format("Strafing (Target: %.1f in)", targetDistance);
        return "Idle";
    }

    public void init(HardwareMap hwMap, MecanumDrive driveTrain) {
        this.driveTrain = driveTrain;
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(2.0, -7.0); // Pod offsets
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
    }
    public double getStartXInches(){
        return startXInches;
    }
    public double getStartYInches(){
        return startYInches;
    }
    public void update(Gamepad gamepad1) {
        pinpoint.update();
        handleManualDistanceInput(gamepad1);
        updateDistanceMovement();
        Log.d(TAG,"X" + getHorizantalValue() + "Error" + getError() + "Target" + getTargetDistance() + "Status" + getStatus() + "Start X" + getStartXInches() + "StrafeMultiplier" + getStrafeMultiplier()+ "addedStrafe" + getAddedStrafe() + "ForwardMultiplier" + getForwardMultiplier() + "AddedForward" + getAddedForward() );
    }

    private Pose2D getPose() {
        pinpoint.update();
        return pinpoint.getPosition();
    }

    public double getHorizantalValue() {
        return getPose().getY(DistanceUnit.INCH);
    }

    public double getVerticalValue() {
        return getPose().getX(DistanceUnit.INCH);
    }

    public double getHeadingDegrees() {
        return getPose().getHeading(AngleUnit.DEGREES);
    }

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

    private void updateDistanceMovement() {
        if (isDrivingY) {
            double distanceTraveled = getVerticalValue() - startYInches;
            error = targetDistance - distanceTraveled;

            if (Math.abs(error) <= DISTANCE_TOLERANCE_INCHES) {
                stopMoving();
            } else {
                double kP = 0.2;
                forwardMultiplier = 0;
                addedForward = -(kP * error);
                if ((Math.min(Math.abs(addedForward),0.8)) != Math.abs(addedForward)){
                    if (error < 0){
                        addedForward = 0.8;
                    } else {
                        addedForward = -0.8;
                    }
                }
                if ((Math.max(Math.abs(addedForward), 0.20)) != Math.abs(addedForward)){
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
                strafeMultiplier = 0;
                addedStrafe = kP * error;
                if ((Math.min(Math.abs(addedStrafe),0.8)) != Math.abs(addedStrafe)){
                    if (error < 0){
                        addedStrafe = -0.8;
                    } else {
                        addedStrafe = 0.8;
                    }
                }
                if ((Math.max(Math.abs(addedStrafe), 0.20)) != Math.abs(addedStrafe)){
                    if (error < 0) {
                        addedStrafe = -0.20;
                    } else {
                        addedStrafe = 0.20;
                    }
                }
                forwardMultiplier = 0;
                addedForward = 0;
            }
        }
        else {
            strafeMultiplier = 1;
            addedStrafe = 0;
            forwardMultiplier = 1;
            addedForward = 0;
        }
    }

    public void driveForwardInches(double distanceInches) {
        startDistanceMovement("forward", distanceInches);
    }

    public void strafeInches(double distanceInches) {
        startDistanceMovement("strafe", distanceInches);
    }

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

        if (gamepad1.right_trigger > 0.3 && !isBusy() && inputDistance != 0) {
            startDistanceMovement(getSelectedDirection(), inputDistance);
        }

        if (gamepad1.right_bumper) {
            stopMoving();
        }
    }

    public void handleDistanceDriving(String direction, double distance) {
        startDistanceMovement(direction, distance);
    }
}
