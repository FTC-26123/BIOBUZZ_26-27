package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AprilTagAlignment {

    private Limelight limelight;
    private ElapsedTime runtime;


    // ------------------------ Rotation Locked ----------------
    double rotationMultiplier = 1;
    double addedRotation = 0;

    // ------------------------ Strafe Locked ------------------
    double strafeMultiplier = 1;
    double addedStrafe = 0;

    // ------------------------ Rotation PD Controller ---------
    double kP_rotation = 0.02;
    double kD_rotation = 0.0;

    double dt = 0;
    double rotationDerivative = 0;

    double error = 0;
    double lastError = 0;

    // ------------------------ Strafe PD Controller -----------
    double kP_strafe = 1.5;
    double kD_strafe = 0.05;

    double strafeError = 0;
    double lastStrafeError = 0;
    double strafeDerivative = 0;
    double strafeOutput = 0;
    double strafeTolerance = 0.02; // 2cm tolerance

    double goalX = 0;
    double angleTolerance = 0.2;

    double curTime = 0;
    double lastTime = 0;

    double[] stepSizes = {1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 2;

    double output = 0;
    double[] modifiableValues = {kP_rotation, kD_rotation, kP_strafe, kD_strafe};
    int modifiableIndex = 0;

    private boolean targetWasVisible = false;

    private static final double MAX_ROTATION_OUTPUT = 1.0;

    /*
    private static final double MAX_STRAFE_OUTPUT = 1.0;
    */

    public void init(Limelight limelight, ElapsedTime runtime){
        this.limelight = limelight;
        this.runtime = runtime;
    }

    public void start() {

        runtime.reset();

        curTime = runtime.time();
        lastTime = curTime;

        error = 0;
        lastError = 0;
        rotationDerivative = 0;
        output = 0;

        /*
        strafeError = 0;
        lastStrafeError = 0;
        strafeDerivative = 0;
        strafeOutput = 0;
        */

        targetWasVisible = false;
    }

    public void update(Gamepad gamepad1) {

        handlePDupdates();
    }

    private void handlePDupdates() {

        lastTime = curTime;
        curTime = runtime.time();

        dt = curTime - lastTime;

        boolean targetVisible = limelight.isTargetVisible();

        if (targetVisible) {

            double newRotationError =
                    goalX - limelight.getHorizontalDelta();

            /*
            double newStrafeError =
                    goalX - limelight.get3DXDistance();
            */

            if (!targetWasVisible) {

                error = newRotationError;
                lastError = newRotationError;
                rotationDerivative = 0;

                /*
                strafeError = newStrafeError;
                lastStrafeError = newStrafeError;
                strafeDerivative = 0;
                */

            } else {

                lastError = error;
                error = newRotationError;

                /*
                lastStrafeError = strafeError;
                strafeError = newStrafeError;
                */

                if (dt > 0.000001) {

                    rotationDerivative =
                            (error - lastError) / dt;

                    /*
                    strafeDerivative =
                            (strafeError - lastStrafeError) / dt;
                    */

                } else {

                    rotationDerivative = 0;

                    /*
                    strafeDerivative = 0;
                    */
                }
            }

            output =
                    (kP_rotation * error)
                            + (kD_rotation * rotationDerivative);

            /*
            strafeOutput =
                    (kP_strafe * strafeError)
                            + (kD_strafe * strafeDerivative);
            */

            output = clamp(
                    output,
                    -MAX_ROTATION_OUTPUT,
                    MAX_ROTATION_OUTPUT
            );

            /*
            strafeOutput = clamp(
                    strafeOutput,
                    -MAX_STRAFE_OUTPUT,
                    MAX_STRAFE_OUTPUT
            );
            */

        } else {

            error = 0;
            lastError = 0;
            rotationDerivative = 0;
            output = 0;

            strafeError = 0;
            lastStrafeError = 0;
            strafeDerivative = 0;
            strafeOutput = 0;
        }

        targetWasVisible = targetVisible;
    }

    public void autoRotate(){
        if (limelight.isTargetVisible()){
            rotationMultiplier = 0;
            if (Math.abs(error) < angleTolerance){
                addedRotation = 0;
            }
            else {
                addedRotation = -output;
            }
        }
        else {
            setManualRotation();
        }
    }

    public void setManualRotation(){
        rotationMultiplier = 1;
        addedRotation = 0;
    }


    public void autoStrafe(){
        if (limelight.isTargetVisible()){
            strafeMultiplier = 0;
            if (Math.abs(strafeError) < strafeTolerance){
                addedStrafe = 0;
            } else {
                addedStrafe = strafeOutput;
            }
        } else {
            setManualStrafing();
        }
    }

    public void setManualStrafing(){
        strafeMultiplier = 1;
        addedStrafe = 0;
    }

    public void addIndexValue(){
        modifiableValues[modifiableIndex] += stepSizes[stepIndex];
        syncModifiableValues();
    }
    public void subIndexValue(){
        modifiableValues[modifiableIndex] -= stepSizes[stepIndex];
        syncModifiableValues();
    }
    public void switchPIDModifyingValue(){
        modifiableIndex = (modifiableIndex + 1) % modifiableValues.length;
    }

    public void switchStepSize(){
        modifiableIndex = (modifiableIndex + 1) % modifiableValues.length;
    }

    private void syncModifiableValues() {

        kP_rotation = modifiableValues[0];
        kD_rotation = modifiableValues[1];
        kP_strafe = modifiableValues[2];
        kD_strafe = modifiableValues[3];
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

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
        String[] names = {"kP_rotation", "kD_rotation", "kP_strafe", "kD_strafe"};
        return names[modifiableIndex];
    }
}

