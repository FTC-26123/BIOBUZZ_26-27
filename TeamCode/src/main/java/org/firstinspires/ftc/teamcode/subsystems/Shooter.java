package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Shooter {

    //Setting up hardware
    private DcMotorEx shooter;
    private Servo gate;
    private Intake intake;


    //Gate positions
    double GATE_OPEN = 0.27;
    double GATE_CLOSED = 0.5;

    //Status variables
    String shooterStatus = "Off";
    String gateStatus = "Closed";

    //Current target speed, zero is default
    double targetLaunchPower = 0;

    public void init(HardwareMap hwMap, Intake intake){
        //Map hardware
        this.intake = intake;
        shooter = hwMap.get(DcMotorEx.class, "launcher");

        gate = hwMap.get(Servo.class, "gate");
        //Set direction
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void stop(){
        //Stop all motion and close the gate
        shooterStatus = "Off";
        shooter.setVelocity(0);
        intake.stop();
        closeGate();
    }
    public void switchSpeed(double speed){
        targetLaunchPower = speed;
    }
    public void openGate(){
        gateStatus = "Open";
        gate.setPosition(GATE_OPEN);
    }
    public void closeGate(){
        gateStatus = "Closed";
        gate.setPosition(GATE_CLOSED);
    }

    public void start(){
        shooterStatus = "Starting";
        shooter.setVelocity(targetLaunchPower);
    }
    public double getCurrentSpeed(){
        return shooter.getVelocity();
    }
    public double getTargetLaunchPower(){
        return targetLaunchPower;
    }
    public String getShooterStatus(){
        return shooterStatus;
    }
    public String getGateStatus(){
        return gateStatus;
    }
    public boolean isReady(){
        return shooter.getVelocity()>=(targetLaunchPower-50);
    }

    public void update(Gamepad gamepad2){
        handleManualShootingControl(gamepad2);
        handleAutoShooting();
        handlePresets(gamepad2);
        handleManualGateControl(gamepad2);
    }
    private void handlePresets(Gamepad gamepad2){
        if(gamepad2.dpad_up){
            //Very far
            switchSpeed(2000);
        }
        if(gamepad2.dpad_right){
            //Far
            switchSpeed(1500);
        }
        if(gamepad2.dpad_down){
            //Close
            switchSpeed(1000);
        }
    }
    private void handleManualShootingControl(Gamepad gamepad2){
        if (gamepad2.right_trigger>0.5) {
            start();
        } else if (gamepad2.right_bumper){
            stop();
        }
    }
    private void handleManualGateControl(Gamepad gamepad2){
        if (gamepad2.x) {
            openGate();
        }
        else if (gamepad2.y) {
            closeGate();
        }
    }
    private void handleAutoShooting(){
        //If motor speed is above/equal to target speed -50 then start the shooting process
        if (targetLaunchPower>0 && isReady()){
            shooterStatus = "Shooting";
            intake.start();
            openGate();
        }
    }
}
