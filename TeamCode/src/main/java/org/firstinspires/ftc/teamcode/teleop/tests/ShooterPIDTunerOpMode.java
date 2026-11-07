package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Automated mathematical PIDF tuner for shooter flywheel.
 * Executes a step response test to identify system parameters (Kv, tau, deadtime)
 * and calculates optimal IMC PIDF gains for 0.1s target response time.
 */
@TeleOp(name = "Shooter PIDF Mathematical Tuner", group = "Diagnostics")
public class ShooterPIDTunerOpMode extends OpMode {

    private DcMotorEx shooter;
    private final ElapsedTime testTimer = new ElapsedTime();

    private enum TestState { IDLE, STEP_TEST, FINISHED }
    private TestState state = TestState.IDLE;

    // Measured system parameters
    private double vMax = 0;
    private double tau = 0;
    private static final double DEAD_TIME_SEC = 0.015; // default 15ms Control Hub delay
    private double tStart = 0;
    private double t63 = 0;

    // Target closed-loop response time (seconds)
    private static final double TARGET_RESPONSE_SEC = 0.10;

    // Calculated optimal PIDF gains
    private double calcKF = 0;
    private double calcKP = 0;
    private double calcKI = 0;
    private double calcKD = 0;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addLine("Press (A) on Gamepad 1 to start mathematical PIDF calibration.");
    }

    @Override
    public void loop() {
        switch (state) {
            case IDLE:
                if (gamepad1.aWasPressed()) {
                    state = TestState.STEP_TEST;
                    testTimer.reset();
                    tStart = testTimer.seconds();
                    shooter.setPower(1.0); // 100% step input
                }
                break;

            case STEP_TEST:
                double currentVel = shooter.getVelocity();
                double time = testTimer.seconds();

                // Track max velocity & 63.2% time constant
                if (currentVel > vMax) {
                    vMax = currentVel;
                }
                if (t63 == 0 && currentVel >= 0.632 * vMax && vMax > 500) {
                    t63 = time;
                }

                // Step test completes after 2.0 seconds of spool-up
                if (time >= 2.0) {
                    shooter.setPower(0);
                    tau = Math.max(0.01, t63 - tStart);
                    calculateOptimalGains();
                    state = TestState.FINISHED;
                }
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
                telemetry.addLine("Press (A) to re-run test.");
                if (gamepad1.aWasPressed()) {
                    vMax = 0;
                    t63 = 0;
                    state = TestState.STEP_TEST;
                    testTimer.reset();
                    shooter.setPower(1.0);
                }
                break;
        }
    }

    private void calculateOptimalGains() {
        if (vMax <= 0) return;
        double denom = vMax * (TARGET_RESPONSE_SEC + DEAD_TIME_SEC);

        calcKF = 1.0 / vMax;
        calcKP = tau / denom;
        calcKI = 1.0 / denom;
        calcKD = (tau * DEAD_TIME_SEC) / denom;
    }
}
