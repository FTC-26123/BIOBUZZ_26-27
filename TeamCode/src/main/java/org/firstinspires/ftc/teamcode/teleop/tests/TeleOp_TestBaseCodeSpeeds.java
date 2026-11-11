package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Locale;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Test TeleOp: every driving and mechanism function from TeleOp_BaseCode (two controllers) and
 * TeleOp_BaseCodeOneController (one controller), plus live speed adjustment of the intake, windmill, transfer and
 * shooter in increments you choose. Nothing here changes the competition code's defaults.
 *
 * <p>CONTROL MODE: press the gamepad 1 LEFT STICK BUTTON to switch the mechanism controller between
 * TWO-CONTROLLER (mechanisms on gamepad 2, like TeleOp_BaseCode) and ONE-CONTROLLER (mechanisms on gamepad 1,
 * like TeleOp_BaseCodeOneController). Driving is always on gamepad 1.
 *
 * <p>Driving (gamepad 1): sticks drive, dpad right = speed multiplier, left trigger = AprilTag auto-rotate and
 * position-hold bypass, right stick button = position-hold bypass.
 * <br>Mechanisms (mechanism pad): left trigger = intake in, back = intake out, left bumper = intake stop,
 * right trigger = shooter start, right bumper = shooter stop, dpad up = next shooter preset.
 *
 * <p>SPEED TUNING (always gamepad 1): X = select next setting, Y = raise it by the increment, A = lower it by the
 * increment, dpad down = bigger increment, dpad left = smaller increment, B = reset the selected setting to its
 * default, START = clear the peak-current readings. Edit the INCREMENTS arrays below to use any increments you like.
 */
@TeleOp
public class TeleOp_TestBaseCodeSpeeds extends OpMode {

    // ---------------------- Increments you can change ----------------------
    private static final double[] POWER_INCREMENTS = {0.01, 0.02, 0.05, 0.10};
    private static final double[] RPM_INCREMENTS = {10, 25, 50, 100, 250};
    private static final double[] SHOOTER_INCREMENTS = {10, 25, 50, 100};

    // ---------------------- Subsystems ----------------------
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final MecanumDrive driveTrain = new MecanumDrive();
    private final ElapsedTime runtime = new ElapsedTime();
    private final Limelight limelight = new Limelight();
    private final AprilTagAlignment aprilTagAlignment = new AprilTagAlignment();

    private boolean twoControllers = true;
    private boolean prevLeftStickButton, prevStart, prevX, prevY, prevA, prevB, prevDpadDown, prevDpadLeft;

    // ---------------------- Tunable settings ----------------------
    private static final class Setting {
        final String name, unit;
        final double[] increments;
        final DoubleSupplier getter;
        final DoubleConsumer setter;
        final Runnable reset;
        final int decimals;
        int incrementIndex = 1;

        Setting(String name, String unit, int decimals, double[] increments, DoubleSupplier getter,
                DoubleConsumer setter, Runnable reset) {
            this.name = name; this.unit = unit; this.decimals = decimals; this.increments = increments;
            this.getter = getter; this.setter = setter; this.reset = reset;
            this.incrementIndex = Math.min(1, increments.length - 1);
        }

        double increment() { return increments[incrementIndex]; }
    }

    private Setting[] settings;
    private int selected = 0;

    @Override
    public void init() {
//        limelight.init(hardwareMap);
        aprilTagAlignment.init(limelight, runtime);
        driveTrain.init(hardwareMap, aprilTagAlignment);
        intake.init(hardwareMap);
        shooter.init(hardwareMap, intake);
        driveTrain.enablePositionHold(hardwareMap);

        settings = new Setting[] {
            new Setting("Intake roller power", "(0-1)", 2, POWER_INCREMENTS,
                intake::getIntakePower, intake::setIntakePower,
                () -> intake.setIntakePower(Intake.INTAKE_POWER)),
            new Setting("Windmill feed speed", "RPM (open loop)", 0, RPM_INCREMENTS,
                intake::getWindmillFeedRpm, intake::setWindmillFeedRpm,
                () -> intake.setWindmillFeedRpm(Double.NaN)),
            new Setting("Windmill reverse while intaking", "RPM (open loop)", 0, RPM_INCREMENTS,
                intake::getWindmillIntakeReverseRpm, intake::setWindmillIntakeReverseRpm,
                () -> intake.setWindmillIntakeReverseRpm(Intake.WINDMILL_INTAKE_REVERSE_RPM)),
            new Setting("Transfer feed speed", "RPM", 0, RPM_INCREMENTS,
                intake::getTransferFeedRpm, intake::setTransferFeedRpm,
                () -> intake.setTransferFeedRpm(Intake.FEED_RPM)),
            new Setting("Transfer power while intaking", "(0-1)", 2, POWER_INCREMENTS,
                intake::getTransferIntakePower, intake::setTransferIntakePower,
                () -> intake.setTransferIntakePower(Intake.TRANSFER_INTAKE_POWER)),
            new Setting("Shooter flywheel", "ticks/s", 0, SHOOTER_INCREMENTS,
                shooter::getTargetLaunchPower, v -> shooter.increaseTargetSpeed(v - shooter.getTargetLaunchPower()),
                () -> shooter.increaseTargetSpeed(1500 - shooter.getTargetLaunchPower()))
        };
    }

    @Override
    public void start() {
        runtime.reset();
//        limelight.start();
    }

    @Override
    public void loop() {
        handleControlModeToggle();
        Gamepad mech = twoControllers ? gamepad2 : gamepad1;

        // ---- Driving (same as both base TeleOps) ----
        aprilTagAlignment.update();
        handleAutoRotation(gamepad1);
        handleDrivetrainDpadSpeedSwitching(gamepad1);
        driveTrain.driveWithPositionHold(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, gamepad1.left_trigger > 0.3 || gamepad1.right_stick_button);

        // ---- Mechanisms (same as both base TeleOps) ----
        handleManualIntakeControl(mech);
        handleManualShootingControl(mech);
        handleShooterSpeedControl(mech);

        // ---- Speed tuning ----
        handleSpeedTuning();

        shooter.update();
        showTelemetry();
    }

    @Override
    public void stop() {
        driveTrain.stop();
    }

    // ---------------------- Base-code handlers ----------------------

    private void handleManualShootingControl(Gamepad pad) {
        if (pad.right_trigger > 0.5) {
            shooter.start();
        } else if (pad.right_bumper) {
            shooter.stop();
        }
    }

    private void handleShooterSpeedControl(Gamepad pad) {
        if (pad.dpadUpWasPressed()) {
            shooter.switchSpeed();
        }
    }

    private void handleManualIntakeControl(Gamepad pad) {
        if (pad.left_trigger > 0.5) {
            intake.start();
        } else if (pad.back) {
            intake.out();
        } else if (pad.left_bumper) {
            intake.stop();
        }
    }

    private void handleDrivetrainDpadSpeedSwitching(Gamepad pad) {
        if (pad.dpadRightWasPressed()) {
            driveTrain.switchSpeed();
        }
    }

    private void handleAutoRotation(Gamepad pad) {
        if (pad.left_trigger > 0.3) {
            aprilTagAlignment.autoRotate();
        } else {
            aprilTagAlignment.setManualRotation();
        }
    }

    // ---------------------- Test-only handlers ----------------------

    private void handleControlModeToggle() {
        boolean now = gamepad1.left_stick_button;
        if (now && !prevLeftStickButton) twoControllers = !twoControllers;
        prevLeftStickButton = now;
    }

    private void handleSpeedTuning() {
        boolean x = gamepad1.x, y = gamepad1.y, a = gamepad1.a, b = gamepad1.b, down = gamepad1.dpad_down,
                left = gamepad1.dpad_left, start = gamepad1.start;
        Setting s = settings[selected];

        if (x && !prevX) selected = (selected + 1) % settings.length;
        if (down && !prevDpadDown) s.incrementIndex = Math.min(s.increments.length - 1, s.incrementIndex + 1);
        if (left && !prevDpadLeft) s.incrementIndex = Math.max(0, s.incrementIndex - 1);
        if (y && !prevY) s.setter.accept(s.getter.getAsDouble() + s.increment());
        if (a && !prevA) s.setter.accept(Math.max(0, s.getter.getAsDouble() - s.increment()));
        if (b && !prevB) s.reset.run();
        if (start && !prevStart) intake.resetPeakCurrent();

        prevX = x; prevY = y; prevA = a; prevB = b; prevDpadDown = down; prevDpadLeft = left; prevStart = start;
    }

    // ---------------------- Telemetry ----------------------

    private String valueText(Setting s, int index) {
        double v = s.getter.getAsDouble();
        String text = String.format(Locale.US, "%." + s.decimals + "f %s", v, s.unit);
        if (index == 1 && intake.isWindmillFollowingFlywheel()) text += "  [AUTO: 90% of flywheel]";
        return text;
    }

    private void showTelemetry() {
        telemetry.addData("Controls", twoControllers
                ? "TWO controllers: mechanisms on gamepad 2 (G1 left stick button to switch)"
                : "ONE controller: mechanisms on gamepad 1 (G1 left stick button to switch)");
        telemetry.addLine("------------- SPEED TUNING (gamepad 1) -------------");
        for (int i = 0; i < settings.length; i++) {
            Setting s = settings[i];
            telemetry.addData((i == selected ? "> " : "  ") + s.name,
                    "%s   (step %s)", valueText(s, i), formatIncrement(s));
        }
        telemetry.addLine("X next setting | Y raise | A lower | dpad down bigger step | dpad left smaller step | B reset to default");
        telemetry.addLine("---------------- Statuses ----------------");
        telemetry.addData("Robot speed multiplier (G1 dpad right)", driveTrain.getSpeedMultiplier());
        telemetry.addData("Shooter status (RT start / RB stop)", shooter.getShooterStatus());
        telemetry.addData("Intake status (LT in / Back out / LB stop)", intake.getIntakeStatus());
        telemetry.addData("Shooter target / actual (ticks/s)", "%.0f / %.0f",
                shooter.getTargetLaunchPower(), shooter.getCurrentSpeed());
        telemetry.addData("Feed step (CLEAR > WINDMILL > FEED)", shooter.getFeedStage());
        telemetry.addData("Windmill power / measured RPM", "%.2f / %.0f",
                intake.getWindmillPower(), intake.getWindmillMeasuredRpm());
        telemetry.addData("Windmill current now / peak (A)", "%.1f / %.1f",
                intake.getWindmillCurrentAmps(), intake.getWindmillPeakCurrentAmps());
        telemetry.addData("Transfer current now / peak (A)", "%.1f / %.1f",
                intake.getTransferCurrentAmps(), intake.getTransferPeakCurrentAmps());
        telemetry.addData("Transfer backed off (overcurrent)", intake.isTransferOvercurrentCooling());
        telemetry.addData("Transfer waiting for windmill", intake.isTransferPaused());
        telemetry.addLine("Press G1 START to clear the peak currents");
        telemetry.addData("Position hold (G1 right stick button bypasses)", driveTrain.getPositionHoldStatus());
        telemetry.addData("Runtime", "%.1f", runtime.seconds());
        telemetry.setMsTransmissionInterval(30);
        telemetry.update();
    }

    private String formatIncrement(Setting s) {
        return s.increment() < 1 ? String.format(Locale.US, "%.2f", s.increment())
                : String.format(Locale.US, "%.0f", s.increment());
    }
}
