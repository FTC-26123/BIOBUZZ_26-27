package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystems.AutoShot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Single-controller TeleOp OpMode with dynamic shooter target speed adjustment.
 * Maps driving, drive speed switching, intake, shooter flywheel, gate servo,
 * preset speed switching, and trimming the shot speed of each preset onto Gamepad 1.
 */
@TeleOp
public class TeleOp_BaseCodeOneControllerSpeedAdjust extends OpMode {

    // ---------------------- Subsystems & Hardware Objects ----------------------
    public Shooter shooter = new Shooter();
    public Intake intake = new Intake();
    public MecanumDrive driveTrain = new MecanumDrive();
    public ElapsedTime TeleOpRuntime = new ElapsedTime();
    public Limelight limelight = new Limelight();
    public AprilTagAlignment aprilTagAlignment = new AprilTagAlignment();
    /** Camera aiming + distance-based shot speed. Does nothing until ShotConfig.AUTO_SHOT_ENABLED is true. */
    public AutoShot autoShot = new AutoShot();

    @Override
    public void init() {
        aprilTagAlignment.init(limelight, TeleOpRuntime);
        autoShot.init(hardwareMap, limelight, aprilTagAlignment, shooter, TeleOpRuntime);
        driveTrain.init(hardwareMap, aprilTagAlignment);
        intake.init(hardwareMap);
        shooter.init(hardwareMap, intake);
        driveTrain.enablePositionHold(hardwareMap);
    }

    /**
     * While waiting for Play: pick the alliance for camera aiming (gamepad 1 X = blue, B = red).
     */
    @Override
    public void init_loop() {
        autoShot.selectAlliance(gamepad1, telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        TeleOpRuntime.reset();
        autoShot.start();
    }

    @Override
    public void loop() {
        // Vision alignment & auto rotation
        boolean aim = aimButton();
        autoShot.update(aim);
        aprilTagAlignment.update();
        handleAutoRotation(aim);

        // Gamepad 1 Drivetrain Control
        handleDrivetrainDpadSpeedSwitching(gamepad1);
        driveTrain.driveWithPositionHold(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, aim || gamepad1.right_stick_button);

        // Gamepad 1 Intake Control
        handleManualIntakeControl(gamepad1);

        // Gamepad 1 Shooter Flywheel & shot preset / trim
        handleManualShootingControl(gamepad1);
        handleShooterSpeedControl(gamepad1);
        shooter.update();

        // ---------------- Telemetry Status Output ----------------
        telemetry.addData("Drive Speed Multiplier (G1 Dpad Right)", driveTrain.getSpeedMultiplier());
        telemetry.addLine("---------------- Statuses ---------------");
        telemetry.addData("Shooter (G1 hold RT fire / Y spin up / RB off)", shooter.getShooterStatus());
        telemetry.addData("Intake Status (G1 LT/Back/LB)", intake.getIntakeStatus());
        telemetry.addData("Shot preset (Dpad Left next, Dpad Up/Dpad Down +/-" + (int) Shooter.TRIM_STEP
                + ", X clear)", shooter.getPresetText());
        telemetry.addData("Flywheel", "%.0f of %.0f ticks/s  %s", shooter.getLastSpeed(),
                shooter.getTargetLaunchPower(), shooter.isReady() ? "READY" : shooter.isRunning() ? "..." : "off");
        if (intake.isJammed()) telemetry.addLine("!! FEED JAMMED - eject with Back, then fire again !!");
        autoShot.addTelemetry(telemetry);
        telemetry.addData("Runtime (s)", "%.2f", TeleOpRuntime.seconds());
        telemetry.setMsTransmissionInterval(30);
        telemetry.addData("Position hold (hold G1 right stick button to bypass)", driveTrain.getPositionHoldStatus());
        telemetry.update();
    }

    @Override
    public void stop() {
        driveTrain.stop();
        autoShot.stop();
    }

    // ---------------------- Single Driver Input Handlers ----------------------

    /**
     * Shooter: HOLD the right trigger to fire (balls feed one at a time, each only when the flywheel is at
     * speed). Releasing it stops feeding but keeps the flywheel spinning for the next volley. Y spins the
     * flywheel up without feeding. Right bumper turns the flywheel off.
     */
    private void handleManualShootingControl(Gamepad gamepad1) {
        if (gamepad1.right_bumper) {
            shooter.stop();
        } else if (gamepad1.right_trigger > 0.5) {
            shooter.start();
        } else {
            shooter.holdFire();   // does nothing unless the shooter was firing
        }
        if (gamepad1.yWasPressed()) {
            shooter.spinUp();
        }
    }

    /**
     * Shot power: Dpad Left = next preset,
     * Dpad Up / Dpad Down = +/- {@link Shooter#TRIM_STEP} ticks/sec on the current preset, X = clear that trim.
     * The trim stays with its preset, so a correction made at one shooting spot is still there next time.
     */
    private void handleShooterSpeedControl(Gamepad gamepad1) {
        if (gamepad1.dpadLeftWasPressed()) {
            shooter.switchSpeed();
        }
        if (gamepad1.dpadUpWasPressed()) {
            shooter.increaseTargetSpeed(Shooter.TRIM_STEP);
        }
        if (gamepad1.dpadDownWasPressed()) {
            shooter.decreaseTargetSpeed(Shooter.TRIM_STEP);
        }
        if (gamepad1.xWasPressed()) {
            shooter.resetTrim();
        }
    }

    private void handleManualIntakeControl(Gamepad gamepad1) {
        if (gamepad1.left_trigger > 0.5) {
            intake.start();
        } else if (gamepad1.back) {
            intake.out();
        } else if (gamepad1.left_bumper) {
            intake.stop();
        }
    }

    private void handleDrivetrainDpadSpeedSwitching(Gamepad gamepad1) {
        if (gamepad1.dpadRightWasPressed()) {
            driveTrain.switchSpeed();
        }
    }

    /**
     * AIM BUTTON. Hold it to turn the robot toward our basket and set the shot speed from the distance.
     * It also bypasses the position hold so the robot is free to turn.
     *
     * TO CHANGE THE BUTTON: edit the one line below. Examples:
     *   gamepad1.left_stick_button      (current: click the left stick)
     *   gamepad1.guide                  (the centre logo button)
     *   gamepad1.dpad_left              (only if nothing else uses it in this OpMode)
     * Do NOT use the left trigger here: it is the intake button in one-controller mode.
     */
    private boolean aimButton() {
        return gamepad1.left_stick_button;
    }

    private void handleAutoRotation(boolean aim) {
        if (aim) {
            aprilTagAlignment.autoRotate();
        } else {
            aprilTagAlignment.setManualRotation();
        }
    }
}

