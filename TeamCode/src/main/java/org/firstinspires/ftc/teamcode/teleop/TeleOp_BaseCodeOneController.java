package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystems.AutoShot;
import org.firstinspires.ftc.teamcode.subsystems.FlowerArm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Leds;
import org.firstinspires.ftc.teamcode.subsystems.MatchTime;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Single-controller TeleOp OpMode for solo driver operation.
 * Maps driving, speed switching, intake, shooter flywheel, gate servo, and vision alignment onto Gamepad 1.
 *
 * <p>Shared code only: run the Blue or Red version (TeleOp_Blue_* / TeleOp_Red_*), which fixes the alliance.
 */
public abstract class TeleOp_BaseCodeOneController extends OpMode {

    /** true = blue alliance, false = red. Set by the Blue / Red TeleOp files. */
    protected abstract boolean blueAlliance();

    // ---------------------- Subsystems & Hardware Objects ----------------------
    public Shooter shooter = new Shooter();
    public Intake intake = new Intake();
    public MecanumDrive driveTrain = new MecanumDrive();
    public ElapsedTime TeleOpRuntime = new ElapsedTime();
    public Limelight limelight = new Limelight();
    public AprilTagAlignment aprilTagAlignment = new AprilTagAlignment();
    /** Camera aiming + distance-based shot speed. Does nothing until ShotConfig.AUTO_SHOT_ENABLED is true. */
    public AutoShot autoShot = new AutoShot();
    /** Servo arm for the flower bed. Does nothing until FlowerArm.ENABLED is true. */
    public FlowerArm flowerArm = new FlowerArm();
    /** Blinkin status lights ("led"). Colour meanings are listed in Leds.java. */
    public Leds leds = new Leds();

    /**
     * Initializes hardware mapping and subsystem dependencies.
     */
    @Override
    public void init() {
        aprilTagAlignment.init(limelight, TeleOpRuntime);
        autoShot.init(hardwareMap, limelight, aprilTagAlignment, shooter, TeleOpRuntime);
        autoShot.lockAlliance(blueAlliance());
        driveTrain.init(hardwareMap, aprilTagAlignment);
        intake.init(hardwareMap);
        flowerArm.init(hardwareMap);   // does not move the arm
        leds.init(hardwareMap);        // does nothing if the Blinkin is missing
        shooter.init(hardwareMap, intake);
        driveTrain.enablePositionHold(hardwareMap);
    }

    /**
     * While waiting for Play: pick the alliance for camera aiming (gamepad 1 X = blue, B = red).
     */
    @Override
    public void init_loop() {
        autoShot.selectAlliance(gamepad1, telemetry);
        leds.showInit(autoShot, flowerArm, TeleOpRuntime.seconds());
        leds.addTelemetry(telemetry);
        telemetry.update();
    }

    /**
     * Resets runtime timer and starts Limelight vision processing upon pressing Play.
     */
    @Override
    public void start() {
        TeleOpRuntime.reset();
        autoShot.start();
    }

    /**
     * Main TeleOp frame loop handling single-driver inputs, subsystem updates, and telemetry output.
     */
    @Override
    public void loop() {
        // AUTO END: TeleOp is over at MatchTime.TELEOP_SEC (120 s) - stop everything and end the program.
        if (MatchTime.over(TeleOpRuntime.seconds(), MatchTime.TELEOP_SEC)) {
            stop();
            requestOpModeStop();
            return;
        }
        // Limelight vision & AprilTag alignment updates
        boolean aim = aimButton();
        autoShot.update(aim);
        aprilTagAlignment.update();
        handleAutoRotation(aim);
        // handleAutoStrafe(gamepad1);

        // Gamepad 1 Mecanum Drivetrain Control
        handleDrivetrainDpadSpeedSwitching(gamepad1);
        driveTrain.driveWithPositionHold(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, gamepad1.right_stick_button, aim);

        // Gamepad 1 Intake Control
        handleManualIntakeControl(gamepad1);

        // Gamepad 1 Shooter Flywheel Control
        handleManualShootingControl(gamepad1);
        handleShooterSpeedControl();
        shooter.update();
        handleFlowerArm();
        flowerArm.update();   // stops the arm at its touch sensors (must run every loop)
        leds.showTeleOp(autoShot, shooter, intake, flowerArm, aim, TeleOpRuntime.seconds());

        // ---------------- Telemetry Status Output ----------------
        telemetry.addData("Robot Speed Multiplier G1 Dpad Right", driveTrain.getSpeedMultiplier());

        telemetry.addData("Shooter (G1 hold RT fire / Y spin up / RB off)", shooter.getShooterStatus());
        telemetry.addData("Intake Status (G1 LT/Back/LB)", intake.getIntakeStatus());
        telemetry.addData("Shot preset (Dpad Up next, Dpad Down prev, B/A +/-" + (int) Shooter.TRIM_STEP
                + ", X clear)", shooter.getPresetText());
        telemetry.addData("Flywheel", "%.0f of %.0f ticks/s  %s", shooter.getLastSpeed(),
                shooter.getTargetLaunchPower(), shooter.isReady() ? "READY" : shooter.isRunning() ? "..." : "off");
        if (intake.isJammed()) telemetry.addLine("!! FEED JAMMED - eject with Back, then fire again !!");
        flowerArm.addTelemetry(telemetry, "G1 Dpad Left toggles");
        autoShot.addTelemetry(telemetry);

        telemetry.addData("Runtime:", TeleOpRuntime.seconds());
        telemetry.setMsTransmissionInterval(30);
        telemetry.addData("Position hold (hold G1 right stick button to bypass)", driveTrain.getPositionHoldStatus());
        telemetry.update();
    }

    @Override
    public void stop() {
        driveTrain.stop();
        flowerArm.stop();
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
     * Shot power: Dpad Up = next preset, Dpad Down = previous preset,
     * B / A = +/- {@link Shooter#TRIM_STEP} ticks/sec on the current preset, X = clear that trim.
     * The trim stays with its preset, so a correction made at one shooting spot is still there next time.
     */
    private void handleShooterSpeedControl() {
        if (gamepad1.dpadUpWasPressed()) {
            shooter.switchSpeed();
        }
        if (gamepad1.dpadDownWasPressed()) {
            shooter.previousSpeed();
        }
        if (gamepad1.bWasPressed()) {
            shooter.increaseTargetSpeed(Shooter.TRIM_STEP);
        }
        if (gamepad1.aWasPressed()) {
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
     * FLOWER ARM. Dpad Left switches the arm between DOWN (then drive into the flower bed with the intake
     * running) and UP. A and B are the speed trim in this OpMode, so the arm is a single toggle button here.
     *
     * TO CHANGE THE BUTTON: edit the one line below, e.g. gamepad1.guideWasPressed().
     */
    private void handleFlowerArm() {
        if (gamepad1.dpadLeftWasPressed()) {
            flowerArm.toggle();
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

    private void handleAutoStrafe(Gamepad gamepad1) {
        if (gamepad1.left_bumper) {
            aprilTagAlignment.autoStrafe();
        } else {
            aprilTagAlignment.setManualStrafing();
        }
    }
}

