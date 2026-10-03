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

    // ----------------------- Controller based PD tuning --------
    double[] stepSizes = {1.0,0.1,0.01,0.001,0.0001};
    int stepIndex = 2;
    double output = 0;
    double[] modifiableValues = {kP_rotation, kD_rotation, kP_strafe, kD_strafe};
    int modifiableIndex = 0;

    // --------------------------------

    public void init(Limelight limelight, ElapsedTime runtime){
        this.limelight = limelight;
        this.runtime = runtime;
    }

    public void start() {
        runtime.reset();
        curTime = runtime.time();
        lastTime = curTime;
        error = goalX - limelight.getHorizontalDelta();
        lastError = error;
    }
    public void update(Gamepad gamepad1){
        handlePDupdates();
        handleIncrements(gamepad1);
        handleAutoRotation(gamepad1);
        handleAutoStrafe(gamepad1);

    }
    private void handlePDupdates(){
        lastTime = curTime;
        curTime = runtime.time();
        dt = curTime - lastTime;

        if (limelight.isTargetVisible()){
            lastError = error;
            error = goalX - limelight.getHorizontalDelta();
            if (dt > 0) {
                rotationDerivative = (error - lastError) / dt;
            }
            output = (kP_rotation * error) + (kD_rotation * rotationDerivative);

            lastStrafeError = strafeError;
            strafeError = goalX - limelight.get3DXDistance();
            if (dt > 0) {
                strafeDerivative = (strafeError - lastStrafeError) / dt;
            }
            strafeOutput = (kP_strafe * strafeError) + (kD_strafe * strafeDerivative);
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
    }
    private void handleAutoRotation(Gamepad gamepad1){
        if (gamepad1.left_trigger>0.3) {
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
                rotationMultiplier = 1;
                addedRotation = 0;
            }

        } else {
            rotationMultiplier = 1;
            addedRotation = 0;
        }
    }
    private void handleAutoStrafe(Gamepad gamepad1){
        if (gamepad1.left_bumper) {
            if (limelight.isTargetVisible()){
                strafeMultiplier = 0;
                if (Math.abs(strafeError) < strafeTolerance){
                    addedStrafe = 0;
                } else {
                    addedStrafe = strafeOutput;
                }
            } else {
                strafeMultiplier = 1;
                addedStrafe = 0;
            }
        } else {
            strafeMultiplier = 1;
            addedStrafe = 0;
        }
    }
    private void handleIncrements(Gamepad gamepad1){
        if (gamepad1.dpadUpWasPressed()){
            modifiableValues[modifiableIndex] += stepSizes[stepIndex];
            syncModifiableValues();
        }
        if (gamepad1.dpadDownWasPressed()){
            modifiableValues[modifiableIndex] -= stepSizes[stepIndex];
            syncModifiableValues();
        }
        if (gamepad1.dpadLeftWasPressed()){
            modifiableIndex = (modifiableIndex + 1) % modifiableValues.length;
        }
        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
    }
    private void syncModifiableValues() {
        kP_rotation = modifiableValues[0];
        kD_rotation = modifiableValues[1];
        kP_strafe = modifiableValues[2];
        kD_strafe = modifiableValues[3];
    }
    public double getRotationMultiplier(){
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
    public double getkP_rotation(){
        return kP_rotation;
    }
    public double getkD_rotation(){
        return kD_rotation;
    }
    public String getCurrentlyModifying() {
        String[] names = {"kP_rotation", "kD_rotation", "kP_strafe", "kD_strafe"};
        return names[modifiableIndex];
    }
}

