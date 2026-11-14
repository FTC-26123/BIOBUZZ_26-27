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
import org.firstinspires.ftc.teamcode.subsystems.ShotConfig;
import org.firstinspires.ftc.teamcode.subsystems.ShotModel;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.TargetTracker;

import java.util.Locale;

/**
 * Tunes the aim point and the distance -> flywheel speed on the field. Everything is saved on the robot and used
 * by TeleOp and autonomous straight away. Needs "Camera enabled" ON in Robot Settings.
 *
 * <p>Gamepad 1 = same as TeleOp_BaseCode (drive, hold LT = aim).
 * <p>Gamepad 2 = same shooter/intake buttons as TeleOp_BaseCode (RT fire, Y spin up, RB off, Dpad Up/Down preset,
 * Dpad Right/Left trim, X clear trim, LT/Back/LB intake), plus:
 * <ul>
 *   <li>Right stick up / down = aim HIGHER / LOWER in the CELL opening (0.05 per push)</li>
 *   <li>Right stick button = SCALE the shot model so it gives the current speed at this distance</li>
 *   <li>A = RECORD a table row: distance now + flywheel target now (corrects the model here)</li>
 *   <li>B = DELETE the row nearest to the distance now</li>
 *   <li>Hold Back + Start for 2 s = delete ALL rows</li>
 * </ul>
 */
@TeleOp(name = "Shot Table Builder", group = "Setup")
public class TeleOp_ShotTableBuilder extends OpMode {

    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final MecanumDrive driveTrain = new MecanumDrive();
    private final Limelight limelight = new Limelight();
    private final AprilTagAlignment aprilTagAlignment = new AprilTagAlignment();
    private final AutoShot autoShot = new AutoShot();
    private final ElapsedTime runtime = new ElapsedTime();

    private String message = "";
    private final ElapsedTime clearTimer = new ElapsedTime();
    private boolean clearHeld, clearDone, stickUp, stickDown;

    @Override
    public void init() {
        aprilTagAlignment.init(limelight, runtime);
        autoShot.init(hardwareMap, limelight, aprilTagAlignment, shooter, runtime);
        driveTrain.init(hardwareMap, aprilTagAlignment);
        intake.init(hardwareMap);
        shooter.init(hardwareMap, intake);
        driveTrain.enablePositionHold(hardwareMap);
        message = ShotConfig.AUTO_SHOT_ENABLED ? "" : "Camera is OFF: turn on 'Camera enabled' in Robot Settings";
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

        // ---------------- Gamepad 2: intake + shooter (as TeleOp_BaseCode) ----------------
        if (gamepad2.left_trigger > 0.5) intake.start();
        else if (gamepad2.back && !gamepad2.start) intake.out();
        else if (gamepad2.left_bumper) intake.stop();

        if (gamepad2.right_bumper) shooter.stop();
        else if (gamepad2.right_trigger > 0.5) shooter.start();
        else shooter.holdFire();
        if (gamepad2.yWasPressed()) shooter.spinUp();
        if (gamepad2.dpadUpWasPressed()) shooter.switchSpeed();
        if (gamepad2.dpadDownWasPressed()) shooter.previousSpeed();
        if (gamepad2.dpadRightWasPressed()) shooter.increaseTargetSpeed(Shooter.TRIM_STEP);
        if (gamepad2.dpadLeftWasPressed()) shooter.decreaseTargetSpeed(Shooter.TRIM_STEP);
        if (gamepad2.xWasPressed()) shooter.resetTrim();
        shooter.update();

        TargetTracker t = autoShot.getTracker();
        boolean haveDistance = t.hasDistance();
        double distance = haveDistance ? t.getDistanceInches() : Double.NaN;
        double height = t.getAimHeightInches();
        double speedNow = shooter.getTargetLaunchPower();

        // ---------------- Gamepad 2: aim point height (right stick up/down, one step per push) ----------------
        boolean up = gamepad2.right_stick_y < -0.6, down = gamepad2.right_stick_y > 0.6;
        if (up && !stickUp) changeHeight(+0.05);
        if (down && !stickDown) changeHeight(-0.05);
        stickUp = up;
        stickDown = down;

        // ---------------- Gamepad 2: model scale and table ----------------
        if (gamepad2.rightStickButtonWasPressed()) {
            if (!haveDistance || !t.isCellUp()) {
                message = "Cannot scale: aim at an UP cell first (hold G1 LT)";
            } else if (ShotModel.scaleTo(speedNow, distance, height)) {
                message = RobotSettings.save() ? String.format(Locale.US, "Model scaled to %.0f here (saved)", speedNow)
                        : RobotSettings.problem();
            } else {
                message = "Model has no answer here: check launch angle / exit height";
            }
        }
        if (gamepad2.aWasPressed()) {
            if (!haveDistance) {
                message = "Cannot record: no camera distance (hold G1 LT to aim)";
            } else {
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
        boolean both = gamepad2.back && gamepad2.start;
        if (both && !clearHeld) { clearTimer.reset(); clearDone = false; }
        clearHeld = both;
        if (both && !clearDone) {
            if (clearTimer.seconds() >= 2.0) {
                clearDone = true;
                RobotSettings.clearShotTable();
                autoShot.reloadTable();
                message = "All rows deleted";
            } else {
                message = String.format(Locale.US, "Keep holding to delete ALL rows... %.1f", 2.0 - clearTimer.seconds());
            }
        }

        // ---------------- Telemetry (kept short) ----------------
        if (t.isVisible()) {
            telemetry.addLine(String.format(Locale.US, "CELL %d-%d %s  %d tags  aim err %+.1f%s",
                    t.getBasketFirstTag(), t.getBasketFirstTag() + 3, t.isCellUp() ? "UP" : "DOWN (not a target)",
                    t.getTagCount(), t.getAimErrorDeg(), autoShot.isAimed() ? "  AIMED" : ""));
        } else {
            telemetry.addLine("CELL: none in view (point the camera at our UP cell, hold G1 LT)");
        }
        telemetry.addLine(haveDistance
                ? String.format(Locale.US, "Dist %.0f in (tags %.0f)  aim height %.1f in", distance,
                t.getClusterDistanceInches(), height)
                : "Dist: unknown");
        double model = haveDistance ? ShotModel.modelSpeed(distance, height) : Double.NaN;
        double fin = haveDistance ? autoShot.autoSpeed() : Double.NaN;
        telemetry.addLine(String.format(Locale.US, "Speed now %.0f  model %s  final %s", speedNow, fmt(model), fmt(fin)));
        telemetry.addLine(String.format(Locale.US, "Height in opening %.2f (G2 RS up/dn)  scale %.2f (G2 RS click)",
                ShotConfig.AIM_HEIGHT_FRACTION, ShotConfig.TICKS_PER_IN_PER_SEC));
        telemetry.addLine("Rows (G2 A add, B delete): " + (ShotConfig.SHOT_TABLE.length == 0 ? "none"
                : RobotSettings.tableText(ShotConfig.SHOT_TABLE).replace(";", "  ")));
        if (!message.isEmpty()) telemetry.addLine(message);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        driveTrain.stop();
        autoShot.stop();
    }

    private void changeHeight(double delta) {
        ShotConfig.AIM_HEIGHT_FRACTION = Math.max(0, Math.min(1,
                Math.round((ShotConfig.AIM_HEIGHT_FRACTION + delta) * 100) / 100.0));
        message = RobotSettings.save()
                ? String.format(Locale.US, "Aim height in opening %.2f (saved)", ShotConfig.AIM_HEIGHT_FRACTION)
                : RobotSettings.problem();
    }

    private static String fmt(double v) {
        return Double.isNaN(v) ? "-" : String.format(Locale.US, "%.0f", v);
    }
}
