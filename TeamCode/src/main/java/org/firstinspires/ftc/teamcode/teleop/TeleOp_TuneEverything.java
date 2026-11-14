package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystems.AutoShot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.SettingsMenu;
import org.firstinspires.ftc.teamcode.subsystems.ShotConfig;
import org.firstinspires.ftc.teamcode.subsystems.ShotModel;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.TargetTracker;
import org.firstinspires.ftc.teamcode.subsystems.TeleOpPositionHold;

import java.util.Locale;

/**
 * Drive, shoot and tune EVERYTHING in one OpMode. Every change is live and saved on the robot at once.
 *
 * <p>Gamepad 1 = same as TeleOp_BaseCode: sticks drive, Dpad Right = drive speed, hold LT = AIM (spot locked,
 * turns to the aim point), right stick button = position hold off.
 *
 * <p>Gamepad 2, always: hold RT = fire, LT = intake, Back = eject, Y = spin up, START = switch SHOOT / TUNE mode.
 * <ul>
 *   <li>SHOOT mode: RB = flywheel off, LB = intake stop, Dpad Up/Down = preset, Dpad Right/Left = trim,
 *       X = clear trim, A = record a shot-table row here, B = delete the nearest row,
 *       right stick up/down = aim higher/lower in the opening, right stick button = scale the shot model here,
 *       left stick button = clear the hold's peak-push number.</li>
 *   <li>TUNE mode: the settings menu. LB/RB = group, Dpad Up/Down = pick, Dpad Right/Left = change,
 *       A/B = step size, X = code default. Changes apply immediately (shooter PIDF, speeds, gate, intake,
 *       aim, aim point, shot model, position hold). Autonomous and flower arm settings apply next run.</li>
 * </ul>
 */
@TeleOp(name = "Tune Everything (drive + shoot + settings)", group = "Setup")
public class TeleOp_TuneEverything extends OpMode {

    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final MecanumDrive driveTrain = new MecanumDrive();
    private final Limelight limelight = new Limelight();
    private final AprilTagAlignment aprilTagAlignment = new AprilTagAlignment();
    private final AutoShot autoShot = new AutoShot();
    private final ElapsedTime runtime = new ElapsedTime();

    private SettingsMenu menu;
    private boolean tuneMode = false;
    private boolean stickUp, stickDown;
    private String message = "";

    @Override
    public void init() {
        aprilTagAlignment.init(limelight, runtime);
        autoShot.init(hardwareMap, limelight, aprilTagAlignment, shooter, runtime);
        driveTrain.init(hardwareMap, aprilTagAlignment);
        intake.init(hardwareMap);
        shooter.init(hardwareMap, intake);
        driveTrain.enablePositionHold(hardwareMap);
        menu = new SettingsMenu(5);
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
        // ---------------- Gamepad 1: driving (as TeleOp_BaseCode) ----------------
        boolean aim = gamepad1.left_trigger > 0.3;
        autoShot.update(aim);
        aprilTagAlignment.update();
        if (aim) aprilTagAlignment.autoRotate(); else aprilTagAlignment.setManualRotation();
        if (gamepad1.dpadRightWasPressed()) driveTrain.switchSpeed();
        driveTrain.driveWithPositionHold(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, gamepad1.right_stick_button, aim);

        // ---------------- Gamepad 2: always ----------------
        if (gamepad2.startWasPressed()) {
            tuneMode = !tuneMode;
            message = "";
        }
        if (gamepad2.left_trigger > 0.5) intake.start();
        else if (gamepad2.back) intake.out();
        else if (!tuneMode && gamepad2.left_bumper) intake.stop();

        if (!tuneMode && gamepad2.right_bumper) shooter.stop();
        else if (gamepad2.right_trigger > 0.5) shooter.start();
        else shooter.holdFire();
        if (gamepad2.yWasPressed()) shooter.spinUp();

        TargetTracker t = autoShot.getTracker();
        if (tuneMode) {
            if (menu.handle(gamepad2) != null) applyLive();
            drainShootButtons();   // so presses made in TUNE mode do not fire when going back to SHOOT
        } else {
            shootModeButtons(t);
        }
        shooter.update();

        // ---------------- Telemetry (short) ----------------
        if (tuneMode) {
            telemetry.addLine("TUNE mode (G2 Start = back to SHOOT)");
            menu.show(telemetry);
            telemetry.addLine(menu.message());
        } else {
            telemetry.addLine("SHOOT mode (G2 Start = TUNE settings)");
        }
        telemetry.addLine(String.format(Locale.US, "Flywheel %.0f of %.0f %s  %s", shooter.getLastSpeed(),
                shooter.getTargetLaunchPower(), shooter.isReady() ? "READY" : shooter.isRunning() ? "..." : "off",
                shooter.getPresetText()));
        autoShot.addTelemetry(telemetry);
        TeleOpPositionHold hold = driveTrain.getPositionHold();
        if (hold != null) {
            telemetry.addLine(String.format(Locale.US, "Hold: %s  off %.1f in  peak %.1f in",
                    hold.getStatus(), hold.getOffsetInches(), hold.getPeakOffsetInches()));
        } else {
            telemetry.addLine("Hold: " + driveTrain.getPositionHoldStatus());
        }
        if (!tuneMode) {
            telemetry.addLine(String.format(Locale.US, "Aim height %.2f  scale %.2f  rows %d",
                    ShotConfig.AIM_HEIGHT_FRACTION, ShotConfig.TICKS_PER_IN_PER_SEC, ShotConfig.SHOT_TABLE.length));
            if (!message.isEmpty()) telemetry.addLine(message);
        }
        telemetry.update();
    }

    /** SHOOT mode gamepad 2 buttons (presets, trim, shot table, aim height, model scale). */
    private void shootModeButtons(TargetTracker t) {
        if (gamepad2.dpadUpWasPressed()) shooter.switchSpeed();
        if (gamepad2.dpadDownWasPressed()) shooter.previousSpeed();
        if (gamepad2.dpadRightWasPressed()) shooter.increaseTargetSpeed(Shooter.TRIM_STEP);
        if (gamepad2.dpadLeftWasPressed()) shooter.decreaseTargetSpeed(Shooter.TRIM_STEP);
        if (gamepad2.xWasPressed()) shooter.resetTrim();

        boolean have = t.hasDistance();
        double distance = t.getDistanceInches(), height = t.getAimHeightInches();
        double speedNow = shooter.getTargetLaunchPower();

        boolean up = gamepad2.right_stick_y < -0.6, down = gamepad2.right_stick_y > 0.6;
        if ((up && !stickUp) || (down && !stickDown)) {
            ShotConfig.AIM_HEIGHT_FRACTION = Math.max(0, Math.min(1,
                    Math.round((ShotConfig.AIM_HEIGHT_FRACTION + (up ? 0.05 : -0.05)) * 100) / 100.0));
            message = saved(String.format(Locale.US, "Aim height %.2f", ShotConfig.AIM_HEIGHT_FRACTION));
        }
        stickUp = up;
        stickDown = down;

        if (gamepad2.rightStickButtonWasPressed()) {
            if (!have || !t.isCellUp()) message = "Cannot scale: aim at an UP cell first (hold G1 LT)";
            else if (ShotModel.scaleTo(speedNow, distance, height)) {
                message = saved(String.format(Locale.US, "Model scaled to %.0f here", speedNow));
            } else message = "Model has no answer here: check launch angle / exit height";
        }
        if (gamepad2.aWasPressed()) {
            if (!have) message = "Cannot record: no camera distance (hold G1 LT to aim)";
            else {
                boolean ok = RobotSettings.addShotRow(distance, speedNow);
                autoShot.reloadTable();
                message = ok ? String.format(Locale.US, "Recorded %.0f in -> %.0f (saved)", distance, speedNow)
                        : RobotSettings.problem();
            }
        }
        if (gamepad2.bWasPressed()) {
            double[] removed = RobotSettings.removeNearestShotRow(distance);
            autoShot.reloadTable();
            message = removed == null ? "Table is already empty"
                    : String.format(Locale.US, "Deleted %.0f in -> %.0f", removed[0], removed[1]);
        }
        if (gamepad2.leftStickButtonWasPressed()) {
            TeleOpPositionHold hold = driveTrain.getPositionHold();
            if (hold != null) hold.resetPeak();
            message = "Hold peak cleared";
        }
    }

    private void drainShootButtons() {
        gamepad2.dpadUpWasPressed(); gamepad2.dpadDownWasPressed();
        gamepad2.dpadRightWasPressed(); gamepad2.dpadLeftWasPressed();
        gamepad2.xWasPressed(); gamepad2.aWasPressed(); gamepad2.bWasPressed();
        gamepad2.rightStickButtonWasPressed(); gamepad2.leftStickButtonWasPressed();
    }

    /** Push changed settings into the parts that copy them. */
    private void applyLive() {
        autoShot.applySettings();
        intake.applySettings();
        TeleOpPositionHold hold = driveTrain.getPositionHold();
        if (hold != null) {
            hold.refreshTuning();
            hold.resetPeak();
        }
    }

    private static String saved(String what) {
        return RobotSettings.save() ? what + " (saved)" : RobotSettings.problem();
    }

    @Override
    public void stop() {
        shooter.stop();
        driveTrain.stop();
        autoShot.stop();
    }
}
