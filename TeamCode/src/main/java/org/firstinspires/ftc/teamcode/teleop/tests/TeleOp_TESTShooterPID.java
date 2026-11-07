package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Automated mathematical PIDF tuner for shooter flywheel.
 * Executes a step response test to identify system parameters (Kv, tau, deadtime)
 * and calculates optimal IMC PIDF gains for 0.1s target response time.
 * Also includes a live validation test mode (Press X) to test calculated gains at 2000 t/s.
 */
@TeleOp(name = "Shooter PIDF Mathematical Tuner", group = "Diagnostics")
public class TeleOp_TESTShooterPID extends OpMode {

    private DcMotorEx shooter;
    private final ElapsedTime testTimer = new ElapsedTime();

    private enum TestState { IDLE, STEP_TEST, VALIDATION_TEST, FINISHED }
    private TestState state = TestState.IDLE;

    // Measured system parameters
    private double vMax = 0;
    private double tau = 0;
    private static final double DEAD_TIME_SEC = 0.015; // default 15ms Control Hub delay
    private double tStart = 0;
    private double t63 = 0;

    // Sample history for precise steady-state time constant calculation
    private final List<Double> sampleTimes = new ArrayList<>();
    private final List<Double> sampleVelocities = new ArrayList<>();

    // Target closed-loop response time (seconds)
    private static final double TARGET_RESPONSE_SEC = 0.10;

    // Validation test target
    private static final double TEST_TARGET_VELOCITY = 2000.0;
    private double validationSpoolTime = 0;

    // Calculated optimal PIDF gains
    private double calcKF = 0;
    private double calcKP = 0;
    private double calcKI = 0;
    private double calcKD = 0;

    // Validation PID state
    private double valIntegralSum = 0;
    private double valLastError = 0;
    private final ElapsedTime valTimer = new ElapsedTime();

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addLine("Press (A) on Gamepad 1 to start step response PIDF calibration.");
    }

    @Override
    public void loop() {
        switch (state) {
            case IDLE:
                if (gamepad1.aWasPressed()) {
                    startStepTest();
                }
                break;

            case STEP_TEST:
                double currentVel = shooter.getVelocity();
                double time = testTimer.seconds();

                sampleTimes.add(time);
                sampleVelocities.add(currentVel);

                if (currentVel > vMax) {
                    vMax = currentVel;
                }

                // Step test completes after 2.0 seconds of spool-up
                if (time >= 2.0) {
                    shooter.setPower(0);
                    processStepTestResults();
                    state = TestState.FINISHED;
                }
                break;

            case VALIDATION_TEST:
                runValidationTest();
                break;

            case FINISHED:
                telemetry.addLine("=== MATHEMATICALLY OPTIMAL PIDF GAINS ===");
                telemetry.addData("Calculated kF", "%.6f", calcKF);
                telemetry.addData("Calculated kP", "%.6f", calcKP);
                telemetry.addData("Calculated kI", "%.6f", calcKI);
                telemetry.addData("Calculated kD", "%.6f", calcKD);
                telemetry.addLine("-----------------------------------------");
                telemetry.addData("Measured Max Velocity (Vmax)", "%.0f t/s", vMax);
                telemetry.addData("Measured Time Constant (tau)", "%.4f s", tau);
                if (validationSpoolTime > 0) {
                    telemetry.addData("Validation Spool Time (to 2000 t/s)", "%.3f s", validationSpoolTime);
                }
                telemetry.addLine("-----------------------------------------");
                telemetry.addLine("Press (A) to re-run step test.");
                telemetry.addLine("Press (X) to run 2000 t/s validation test.");

                if (gamepad1.aWasPressed()) {
                    startStepTest();
                } else if (gamepad1.xWasPressed()) {
                    startValidationTest();
                }
                break;
        }
    }

    private void startStepTest() {
        vMax = 0;
        t63 = 0;
        sampleTimes.clear();
        sampleVelocities.clear();
        state = TestState.STEP_TEST;
        testTimer.reset();
        tStart = testTimer.seconds();
        shooter.setPower(1.0);
    }

    private void processStepTestResults() {
        if (vMax <= 0 || sampleVelocities.isEmpty()) return;

        double target63 = 0.632 * vMax;
        t63 = tStart;

        int size = sampleVelocities.size();
        for (int k = 0; k < size; k++) {
            if (sampleVelocities.get(k) >= target63) {
                t63 = sampleTimes.get(k);
                break;
            }
        }

        tau = Math.max(0.01, t63 - tStart);
        calculateOptimalGains();
    }

    private void calculateOptimalGains() {
        if (vMax <= 0) return;
        double denom = vMax * (TARGET_RESPONSE_SEC + DEAD_TIME_SEC);

        calcKF = 1.0 / vMax;
        calcKP = tau / denom;
        calcKI = 1.0 / denom;
        calcKD = (tau * DEAD_TIME_SEC) / denom;
    }

    private void startValidationTest() {
        valIntegralSum = 0;
        valLastError = 0;
        validationSpoolTime = 0;
        state = TestState.VALIDATION_TEST;
        valTimer.reset();
        testTimer.reset();
    }

    private void runValidationTest() {
        double currentVel = shooter.getVelocity();
        double dt = valTimer.seconds();
        valTimer.reset();
        if (dt <= 0.0001) dt = 0.01;

        double time = testTimer.seconds();
        double error = TEST_TARGET_VELOCITY - currentVel;

        if (validationSpoolTime == 0 && Math.abs(error) <= 50) {
            validationSpoolTime = time;
        }

        double derivative = (error - valLastError) / dt;
        valLastError = error;

        if (Math.abs(error) < 100) {
            valIntegralSum += error * dt;
        }

        double ff = calcKF * TEST_TARGET_VELOCITY;
        double power = ff + (calcKP * error) + (calcKI * valIntegralSum) + (calcKD * derivative);
        power = Math.max(0.0, Math.min(1.0, power));

        shooter.setPower(power);

        telemetry.addLine("=== VALIDATION TEST IN PROGRESS ===");
        telemetry.addData("Target Velocity", "%.0f t/s", TEST_TARGET_VELOCITY);
        telemetry.addData("Current Velocity", "%.0f t/s", currentVel);
        telemetry.addData("Time Elapsed", "%.2f s", time);

        if (time >= 3.0) {
            shooter.setPower(0);
            state = TestState.FINISHED;
        }
    }
}
