package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

/**
 * OpMode providing automated Ziegler-Nichols relay tuning for AprilTag PID constants.
 * Oscillates the robot around target zero error, calculates ultimate period (Tu) and ultimate gain (Ku),
 * and live-loads calculated kP and kD parameters directly into AprilTagAlignment.
 */
@TeleOp
public class TeleOp_AprilTagAlignmentSetupAutomatic extends OpMode {

    // ---------------------- Subsystem Hardware Objects ----------------------
    public Shooter shooter = new Shooter();
    public Intake intake = new Intake();
    public MecanumDrive driveTrain = new MecanumDrive();
    public ElapsedTime TeleOpRuntime = new ElapsedTime();
    public Limelight limelight = new Limelight();
    public AprilTagAlignment aprilTagAlignment = new AprilTagAlignment();

    // ---------------------- Auto-Tuning State Machine ----------------------
    private enum TuningState {IDLE, TUNING_ROT, TUNING_STRAFE, DONE}
    private TuningState tuningState = TuningState.IDLE;

    // ---------------------- Relay Calibration Parameters ----------------------
    private final double TEST_POWER = 0.20; // Constant relay power step (20%)
    private double zeroCrossings = 0;
    private double firstZeroCrossingTime = 0;

    private double currentError = 0;
    private double lastError = 0;
    private double currentMaxDeviance = 0;
    private List<Double> waveDeviances = new ArrayList<>();

    // ---------------------- Calibrated PID Outputs ----------------------
    private double auto_kP_rot = 0;
    private double auto_kD_rot = 0;
    private double auto_kP_str = 1.5;
    private double auto_kD_str = 0.002;

    /**
     * Initializes Limelight vision camera and robot subsystems.
     */
    @Override
    public void init() {
        Log.v(Limelight.TAG, "Initializing limelight");
        limelight.init(hardwareMap);
        aprilTagAlignment.init(limelight, TeleOpRuntime);
        driveTrain.init(hardwareMap, aprilTagAlignment);
        intake.init(hardwareMap);
        shooter.init(hardwareMap, intake);
    }

    /**
     * Starts vision camera feed and resets timers and tuning state machine upon pressing Play.
     */
    @Override
    public void start() {
        TeleOpRuntime.reset();
        limelight.start();
        aprilTagAlignment.start();
        tuningState = TuningState.IDLE;
    }

    /**
     * Main TeleOp loop handling auto-calibration, driver manual controls, and telemetry output.
     */
    @Override
    public void loop() {
        limelight.update();

        // Handle automated Ziegler-Nichols calibration routine
        handleAutoCalibration(gamepad1);

        // Driver 2 Intake Controls
        handleManualIntakeControl(gamepad2);

        // Driver 2 Shooter & Gate Controls
        handleManualShootingControl(gamepad2);
        handleShooterSpeedControl(gamepad2);
        handleManualGateControl(gamepad2);
        shooter.update();

        // ---------------- Telemetry Output ----------------
        telemetry.addData("Robot Speed Multiplier(G1DpadRight)", driveTrain.getSpeedMultiplier());

        telemetry.addLine("---------------- Statuses ---------------");
        telemetry.addData("Shooter Status G2 RT", shooter.getShooterStatus());
        telemetry.addData("Gate Status G2 X/Y", shooter.getGateStatus());
        telemetry.addData("Intake Status G2 LT/Back/LB", intake.getIntakeStatus());
        telemetry.addData("Target Launch Power G2 Dpad Up", shooter.getTargetLaunchPower());
        telemetry.addData("Current Motor Speed", shooter.getCurrentSpeed());

        telemetry.addLine("---------------- Calibration ---------------");
        telemetry.addData("Calibration Status G1 Logo/Guide", tuningState.toString());
        if (auto_kP_rot!=0 && auto_kD_rot!=0){
            telemetry.addData("Tuned kP-Rot", auto_kP_rot);
            telemetry.addData("Tuned kD-Rot", auto_kD_rot);
        }
        if (auto_kP_str!=0 && auto_kD_str!=0 && tuningState == TuningState.DONE){
            telemetry.addData("Tuned kP-Str", auto_kP_str);
            telemetry.addData("Tuned kD-Str", auto_kD_str);
        }
        telemetry.addData("Zero Crossings Count", zeroCrossings);

        telemetry.addLine("---------------- Calculated Parameters ---------------");
        telemetry.addData("Live Loaded kP_Rotation", aprilTagAlignment.getkP_rotation());
        telemetry.addData("Live Loaded kD_Rotation", aprilTagAlignment.getkD_rotation());
        telemetry.addData("Live Loaded kP_Strafe", aprilTagAlignment.getkP_strafe());
        telemetry.addData("Live Loaded kD_Strafe", aprilTagAlignment.getkD_strafe());

//        telemetry.addLine("----------------April Tags ---------------");
//
//        telemetry.addData("HorAngleDelta", limelight.getHorizontalDelta());
//        telemetry.addData("VerAngleDelta", limelight.getVerticalDelta());
//        telemetry.addData("IsValid", limelight.isTargetVisible());

//        Pose3D botPose = limelight.getBotPose();
//        if (botPose != null && limelight.isTargetVisible()) {
//            telemetry.addData("3D X (m)", "%.2f", botPose.getPosition().x);
//            telemetry.addData("3D Y (m)", "%.2f", botPose.getPosition().y);
//            telemetry.addData("3D Z (m)", "%.2f", botPose.getPosition().z);
//            telemetry.addData("Yaw (°)", "%.2f", botPose.getOrientation().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
//            telemetry.addData("Pitch (°)", "%.2f", botPose.getOrientation().getPitch(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
//            telemetry.addData("Roll (°)", "%.2f", botPose.getOrientation().getRoll(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
//        } else {
//            telemetry.addData("3D BotPose", "No 3D Pose Available");
//        }


        telemetry.addData("Runtime:", TeleOpRuntime.seconds());
        telemetry.setMsTransmissionInterval(30);
        telemetry.update();
    }

    // ---------------------- Driver Control Helpers ----------------------

    private void handleManualShootingControl(Gamepad gamepad2) {
        if (gamepad2.right_trigger > 0.5) {
            shooter.start();
        } else if (gamepad2.right_bumper) {
            shooter.stop();
        }
    }

    private void handleShooterSpeedControl(Gamepad gamepad2) {
        if (gamepad2.dpadUpWasPressed()) {
            shooter.switchSpeed();
        }
    }

    private void handleManualGateControl(Gamepad gamepad2) {
        if (gamepad2.x) {
            shooter.openGate();
        } else if (gamepad2.y) {
            shooter.closeGate();
        }
    }

    private void handleManualIntakeControl(Gamepad gamepad2) {
        if (gamepad2.left_trigger > 0.5) {
            intake.start();
        } else if (gamepad2.back) {
            intake.out();
        } else if (gamepad2.left_bumper) {
            intake.stop();
        }
    }

    private void handleDrivetrainDpadSpeedSwitching(Gamepad gamepad1) {
        if (gamepad1.dpadRightWasPressed()) {
            driveTrain.switchSpeed();
        }
    }

    private void handleApriltagAlignmentIncrements(Gamepad gamepad1) {
        if (gamepad1.dpadUpWasPressed()) {
            aprilTagAlignment.addIndexValue();
        }
        if (gamepad1.dpadDownWasPressed()) {
            aprilTagAlignment.subIndexValue();
        }
        if (gamepad1.dpadLeftWasPressed()) {
            aprilTagAlignment.switchPIDModifyingValue();
        }
        if (gamepad1.bWasPressed()) {
            aprilTagAlignment.switchStepSize();
        }
    }

    private void handleAutoRotation(Gamepad gamepad1) {
        if (gamepad1.left_trigger > 0.3) {
            aprilTagAlignment.autoRotate();
        } else {
            aprilTagAlignment.setManualRotation();
        }
    }

    private void handleAutoStrafe(Gamepad gamepad1) {
        if (gamepad1.left_bumper) {
            aprilTagAlignment.autoStrafe();
        } else {
            aprilTagAlignment.setManualStrafing();
        }
    }

    /**
     * Resets wave tracking counters before starting a relay test sequence.
     */
    private void clearWaveCounters() {
        zeroCrossings = 0;
        firstZeroCrossingTime = 0;
        currentMaxDeviance = 0;
        waveDeviances.clear();
    }

    /**
     * Executes real-time Ziegler-Nichols relay oscillation testing to determine ultimate period and gain.
     */
    private void handleLiveCalibration() {
        if (!limelight.isTargetVisible()) {
            driveTrain.drive(0, 0, 0);
            return;
        }

        double curTime = TeleOpRuntime.seconds();

        // ---------------- Rotation Calibration ----------------
        if (tuningState == TuningState.TUNING_ROT) {
            currentError = 0 - limelight.getHorizontalDelta();
            double rotDrivetrainValue = (currentError > 0) ? TEST_POWER : -TEST_POWER;
            driveTrain.drive(0, 0, rotDrivetrainValue);

            currentMaxDeviance = Math.max(currentMaxDeviance, Math.abs(currentError));

            // Detect sign change in error (zero crossing)
            if (currentError * lastError < 0 && lastError != 0) {
                zeroCrossings++;

                if (zeroCrossings == 1) {
                    currentMaxDeviance = 0;
                    firstZeroCrossingTime = curTime;
                } else {
                    waveDeviances.add(currentMaxDeviance);
                    currentMaxDeviance = 0;
                }

                // After 7 zero-crossings (3 full wave cycles), compute Tu and Ku constants
                if (zeroCrossings >= 7) {
                    double Tu = (curTime - firstZeroCrossingTime) / ((zeroCrossings - 1) / 2.0);

                    double avgDev = 0;
                    for (double d : waveDeviances) avgDev += d;
                    avgDev /= waveDeviances.size();

                    if (avgDev < 0.01) avgDev = 0.01;

                    // Ziegler-Nichols equations for ultimate gain (Ku) and PD gains (kP, kD)
                    double Ku = (4 * TEST_POWER) / (Math.PI * avgDev);
                    auto_kP_rot = 0.45 * Ku;
                    auto_kD_rot = 0.05 * auto_kP_rot * Tu;

                    aprilTagAlignment.init(limelight, TeleOpRuntime, auto_kP_rot, auto_kD_rot, auto_kP_str, auto_kD_str);

                    tuningState = TuningState.DONE;
                    clearWaveCounters();
                }
            }
            lastError = currentError;
        }
        // ---------------- Strafe Calibration ----------------
        else if (tuningState == TuningState.TUNING_STRAFE) {
            Pose3D botPose = limelight.getBotPose();
            if (botPose == null) return;

            currentError = 0.0 - botPose.getPosition().x;
            double strafeDrivetrainValue = (currentError > 0) ? TEST_POWER : -TEST_POWER;
            driveTrain.drive(0, strafeDrivetrainValue, 0);

            currentMaxDeviance = Math.max(currentMaxDeviance, Math.abs(currentError));

            if (currentError * lastError < 0 && lastError != 0) {
                zeroCrossings++;

                if (zeroCrossings == 1) {
                    currentMaxDeviance = 0;
                    firstZeroCrossingTime = curTime;
                } else {
                    waveDeviances.add(currentMaxDeviance);
                    currentMaxDeviance = 0;
                }

                if (zeroCrossings >= 7) {
                    double Tu = (curTime - firstZeroCrossingTime) / ((zeroCrossings - 1) / 2.0);

                    double avgAmp = 0;
                    for (double d : waveDeviances) avgAmp += d;
                    avgAmp /= waveDeviances.size();
                    if (avgAmp < 0.001) avgAmp = 0.001;

                    double Ku = (4.0 * TEST_POWER) / (Math.PI * avgAmp);

                    auto_kP_str = 0.45 * Ku;
                    auto_kD_str = 0.05 * auto_kP_str * Tu;

                    aprilTagAlignment.init(limelight, TeleOpRuntime, auto_kP_rot, auto_kD_rot, auto_kP_str, auto_kD_str);
                    tuningState = TuningState.DONE;
                    clearWaveCounters();
                }
            }
            lastError = currentError;
        }
    }

    /**
     * Triggers auto-calibration on Gamepad 1 Guide button or processes driver teleop driving when idle.
     */
    private void handleAutoCalibration(Gamepad gamepad1) {
        if (gamepad1.guideWasPressed() && tuningState == TuningState.IDLE) {
            tuningState = TuningState.TUNING_ROT;
            clearWaveCounters();
        }

        if (tuningState == TuningState.TUNING_ROT || tuningState == TuningState.TUNING_STRAFE) {
            handleLiveCalibration();
        } else {
            aprilTagAlignment.update();
            handleDrivetrainDpadSpeedSwitching(gamepad1);
            driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            handleAutoRotation(gamepad1);
            // handleAutoStrafe(gamepad1);
        }
    }
}
