package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystems.AutoShot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Locale;

/**
 * Flywheel tuner, following "PID Control Full Course: Build & Tuning for Robotic Flywheels" (B. Pratt):
 * tune kS, then kV, then kP. Gains start at 0. Nothing is kept until gamepad 2 START (save).
 *
 * <p>Gamepad 1 = same as TeleOp_BaseCode (drive, hold LT = aim).
 * <p>Gamepad 2:
 * <ul>
 *   <li>Dpad Left / Right = step: kS, kV, kP (the flywheel is driven the way that step needs)</li>
 *   <li>Dpad Up / Down = this step's gain + / -.  A = steps x10, B = steps /10</li>
 *   <li>Y = flywheel start / stop.  X = switch target HIGH / LOW (like a ball going through).  RB = next preset</li>
 *   <li>Left stick button (kV step) = kV helper: kV x target / actual, using the last 1 s of speed</li>
 *   <li>Right stick button = all gains to 0.  START = SAVE kS, kV, kP</li>
 *   <li>Hold RT = fire a real ball (kP step).  LT = intake, Back = eject, LB = intake stop</li>
 * </ul>
 * Unsaved changes are thrown away when this OpMode stops.
 */
@TeleOp(name = "Shooter Tuner (kS, kV, kP)", group = "Tuning")
public class TeleOp_ShooterPIDManual extends OpMode {

    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final MecanumDrive driveTrain = new MecanumDrive();
    private final Limelight limelight = new Limelight();
    private final AprilTagAlignment aprilTagAlignment = new AprilTagAlignment();
    private final AutoShot autoShot = new AutoShot();
    private final ElapsedTime runtime = new ElapsedTime();

    private static final double[] BASE_STEP = {0.01, 0.00001, 0.0001};
    /** LOW target = HIGH x this (1500 -> about 800, as in the video). */
    private static final double LOW_FRACTION = 0.55;
    private static final double REACHED = 25;   // ticks/sec counted as "at target"

    private int step = 0;
    private double stepMult = 1;
    private boolean running = false, low = false;
    private String saveMessage = "not saved (gains start at 0)";
    private String helperMessage = "";

    // Step response: from a target change (or a dip, like a ball) until back within REACHED, then overshoot for 1 s.
    private final ElapsedTime responseTimer = new ElapsedTime(), overTimer = new ElapsedTime();
    private boolean responding = false, trackingOver = false;
    private double lastTarget = 0, lastReach = Double.NaN, lastOver = Double.NaN;
    // 1 s speed average for the kV helper.
    private final ElapsedTime avgTimer = new ElapsedTime();
    private double avgSum = 0, avgSpeed = Double.NaN;
    private int avgCount = 0;

    @Override
    public void init() {
        aprilTagAlignment.init(limelight, runtime);
        autoShot.init(hardwareMap, limelight, aprilTagAlignment, shooter, runtime);
        driveTrain.init(hardwareMap, aprilTagAlignment);
        intake.init(hardwareMap);
        shooter.init(hardwareMap, intake);
        driveTrain.enablePositionHold(hardwareMap);
        Shooter.kS = 0;
        Shooter.kF = 0;
        Shooter.kP = 0;
        Shooter.kI = 0;
        Shooter.kD = 0;
        applyStep();
    }

    @Override
    public void init_loop() {
        autoShot.selectAlliance(gamepad1, telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        autoShot.start();
    }

    @Override
    public void loop() {
        // ---------------- Gamepad 1: driving ----------------
        boolean aim = gamepad1.left_trigger > 0.3;
        autoShot.update(aim);
        aprilTagAlignment.update();
        if (aim) aprilTagAlignment.autoRotate(); else aprilTagAlignment.setManualRotation();
        if (gamepad1.dpadRightWasPressed()) driveTrain.switchSpeed();
        driveTrain.driveWithPositionHold(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, gamepad1.right_stick_button, aim);

        // ---------------- Gamepad 2 ----------------
        if (gamepad2.left_trigger > 0.5) intake.start();
        else if (gamepad2.back) intake.out();
        else if (gamepad2.left_bumper) intake.stop();

        if (gamepad2.dpadRightWasPressed()) { step = Math.min(2, step + 1); applyStep(); }
        if (gamepad2.dpadLeftWasPressed()) { step = Math.max(0, step - 1); applyStep(); }
        if (gamepad2.aWasPressed()) stepMult = Math.min(1000, stepMult * 10);
        if (gamepad2.bWasPressed()) stepMult = Math.max(0.001, stepMult / 10);
        if (gamepad2.dpadUpWasPressed()) changeGain(+1);
        if (gamepad2.dpadDownWasPressed()) changeGain(-1);
        if (gamepad2.rightStickButtonWasPressed()) {
            Shooter.kS = 0;
            Shooter.kF = 0;
            Shooter.kP = 0;
            helperMessage = "All gains 0";
        }
        if (gamepad2.yWasPressed()) {
            running = !running;
            if (running) shooter.spinUp(); else shooter.stop();
        }
        if (gamepad2.xWasPressed()) low = !low;
        if (gamepad2.rightBumperWasPressed()) shooter.switchSpeed();
        if (running) {
            if (step == 2 && gamepad2.right_trigger > 0.5) shooter.start(); else shooter.spinUp();
        }
        if (gamepad2.leftStickButtonWasPressed() && step == 1) kvHelper();
        if (gamepad2.startWasPressed()) {
            saveMessage = RobotSettings.save()
                    ? String.format(Locale.US, "SAVED kS %s kV %s kP %s", RobotSettings.trim(Shooter.kS),
                    RobotSettings.trim(Shooter.kF), RobotSettings.trim(Shooter.kP))
                    : RobotSettings.problem();
        }

        // LOW / HIGH: trim the current preset down and back (the preset itself is not changed).
        double high = shooter.getPresetSpeed();
        double wanted = low ? Math.round(high * LOW_FRACTION) : high;
        double now = shooter.getTargetLaunchPower();
        if (Math.abs(now - wanted) > 0.5) shooter.increaseTargetSpeed(wanted - now);

        shooter.update();
        measure();

        // ---------------- Telemetry (short) ----------------
        telemetry.addLine(instruction());
        telemetry.addLine(String.format(Locale.US, "%skS %s   %skV %s   %skP %s   step %s",
                step == 0 ? ">" : " ", RobotSettings.trim(Shooter.kS), step == 1 ? ">" : " ",
                RobotSettings.trim(Shooter.kF), step == 2 ? ">" : " ", RobotSettings.trim(Shooter.kP),
                RobotSettings.trim(stepSize())));
        double target = shooter.getTargetLaunchPower(), speed = shooter.getLastSpeed();
        telemetry.addLine(String.format(Locale.US, "%s  target %.0f %s  speed %.0f  err %+.0f  power %.3f",
                running ? "RUNNING" : "stopped (Y)", target, low ? "LOW" : "HIGH", speed, target - speed,
                shooter.getLastPower()));
        telemetry.addLine(String.format(Locale.US, "Last change: back in %s s, overshoot %s   1 s avg %s",
                fmt2(lastReach), fmt0(lastOver), fmt0(avgSpeed)));
        if (!helperMessage.isEmpty()) telemetry.addLine(helperMessage);
        telemetry.addLine("Save (G2 Start): " + saveMessage);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setTuneMode(Shooter.TuneMode.NORMAL);
        shooter.stop();
        driveTrain.stop();
        autoShot.stop();
        RobotSettings.reload();   // drop unsaved gains so other OpModes use the saved ones
    }

    private void applyStep() {
        shooter.setTuneMode(step == 0 ? Shooter.TuneMode.KS_ONLY
                : step == 1 ? Shooter.TuneMode.FEEDFORWARD_ONLY : Shooter.TuneMode.NORMAL);
        helperMessage = "";
    }

    private double stepSize() { return BASE_STEP[step] * stepMult; }

    private void changeGain(int dir) {
        double d = dir * stepSize();
        if (step == 0) Shooter.kS = Math.max(0, Shooter.kS + d);
        else if (step == 1) Shooter.kF = Math.max(0, Shooter.kF + d);
        else Shooter.kP = Math.max(0, Shooter.kP + d);
        shooter.resetIntegral();
    }

    private String instruction() {
        switch (step) {
            case 0: return "STEP 1 kS: Y = run. Raise kS until the wheel JUST starts turning from stopped. "
                    + "Stop/start (Y) 3 times; use the average.";
            case 1: return "STEP 2 kV: Y = run. Raise kV until speed is just UNDER target (LS = helper). "
                    + "X: LOW should be close too.";
            default: return "STEP 3 kP: Y = run, X toggles LOW/HIGH. Raise kP until it gets back fast; "
                    + "lower if it overshoots or hums. RT = real ball.";
        }
    }

    /** kV x target / actual from the last second of speed (feedforward only, so this lands on the true kV). */
    private void kvHelper() {
        double target = shooter.getTargetLaunchPower();
        if (!running || Double.isNaN(avgSpeed) || target <= 0) {
            helperMessage = "kV helper: run the flywheel for 1 s first (Y)";
            return;
        }
        if (Shooter.kF <= 0 || avgSpeed < 50) {
            Shooter.kF = 1.0 / 2800.0;
            helperMessage = "kV started from 1/2800. Wait 1 s, press again";
            return;
        }
        double old = Shooter.kF;
        Shooter.kF = old * target / avgSpeed;
        helperMessage = String.format(Locale.US, "kV %s -> %s (speed %.0f of %.0f). Wait 1 s, press again",
                RobotSettings.trim(old), RobotSettings.trim(Shooter.kF), avgSpeed, target);
    }

    /** Time to get back within REACHED after a target change or a dip, then the overshoot over the next 1 s. */
    private void measure() {
        double target = shooter.getTargetLaunchPower(), speed = shooter.getLastSpeed();
        double err = target - speed;
        if (running) {
            boolean begin = target != lastTarget || (!responding && !trackingOver && Math.abs(err) > 3 * REACHED);
            if (begin) {
                responding = true;
                trackingOver = false;
                responseTimer.reset();
            }
            if (responding && Math.abs(err) <= REACHED) {
                lastReach = responseTimer.seconds();
                responding = false;
                trackingOver = true;
                overTimer.reset();
                lastOver = 0;
            }
            if (trackingOver) {
                double past = speed - target;
                if (Math.abs(past) > Math.abs(lastOver)) lastOver = past;
                if (overTimer.seconds() > 1.0) trackingOver = false;
            }
        } else {
            responding = trackingOver = false;
        }
        lastTarget = target;

        avgSum += speed;
        avgCount++;
        if (avgTimer.seconds() >= 1.0) {
            avgSpeed = avgSum / Math.max(1, avgCount);
            avgSum = 0;
            avgCount = 0;
            avgTimer.reset();
        }
    }

    private static String fmt2(double v) { return Double.isNaN(v) ? "-" : String.format(Locale.US, "%.2f", v); }

    private static String fmt0(double v) { return Double.isNaN(v) ? "-" : String.format(Locale.US, "%+.0f", v); }
}
