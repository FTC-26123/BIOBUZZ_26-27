package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.AutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.FlowerArm;
import org.firstinspires.ftc.teamcode.subsystems.RobotSettings;

import java.util.Locale;

/**
 * FLOWER ARM TEST + TUNE (continuous servo "flowerArm", touch sensors "touchUp" / "touchDown"). Nothing else moves.
 *
 * <pre>
 * Gamepad 1
 *   Left stick up/down (hold)  jog the arm slowly. It still stops at a pressed switch.
 *   Y = drive UP to touchUp    A = drive DOWN to touchDown    B = stop
 *   X = cycle test: down, up, down, up ... timing every move (X again to stop)
 *   Dpad Up/Down     Up power   +/- 0.05
 *   Dpad Right/Left  Down power +/- 0.05
 *   Back             flip Direction (if "up" moves the arm down)
 *   Start            SAVE the powers / direction / timings to Robot Settings
 * </pre>
 * The switches are shown live even before Play and even with the arm switched off, so they can be checked by
 * pressing them by hand first. Robot Settings > Flower arm > Enabled must be ON for the arm to move.
 */
@TeleOp(name = "Flower Arm Test (touch sensors + tune)", group = "Setup")
public class TeleOp_FlowerArmTest extends OpMode {
    private final FlowerArm arm = new FlowerArm();
    private boolean cycling = false;
    private int cycleMoves = 0;
    private double slowestUp = 0, slowestDown = 0, lastUp = Double.NaN, lastDown = Double.NaN;
    private FlowerArm.State lastState;
    private final ElapsedTime pause = new ElapsedTime();
    private String message = "";

    @Override
    public void init() {
        RobotSettings.ensureLoaded();
        arm.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("=== FLOWER ARM TEST ===  (press PLAY to move it)");
        telemetry.addData("Switches", arm.switchText());
        telemetry.addLine("Press each switch by hand: it must show PRESSED, and only that one.");
        telemetry.addData("Arm", arm.getStatus());
        telemetry.update();
    }

    @Override
    public void loop() {
        // ---- buttons ----
        if (gamepad1.yWasPressed()) { cycling = false; arm.up(); }
        if (gamepad1.aWasPressed()) { cycling = false; arm.down(); }
        if (gamepad1.bWasPressed()) { cycling = false; arm.stop(); }
        if (gamepad1.xWasPressed()) {
            cycling = !cycling;
            cycleMoves = 0;
            slowestUp = slowestDown = 0;
            if (!cycling) arm.stop(); else if (arm.isAtDown()) arm.up(); else arm.down();
        }
        if (gamepad1.dpadUpWasPressed()) FlowerArm.UP_POWER = clamp(FlowerArm.UP_POWER + .05);
        if (gamepad1.dpadDownWasPressed()) FlowerArm.UP_POWER = clamp(FlowerArm.UP_POWER - .05);
        if (gamepad1.dpadRightWasPressed()) FlowerArm.DOWN_POWER = clamp(FlowerArm.DOWN_POWER + .05);
        if (gamepad1.dpadLeftWasPressed()) FlowerArm.DOWN_POWER = clamp(FlowerArm.DOWN_POWER - .05);
        if (gamepad1.backWasPressed()) {
            arm.stop();
            cycling = false;
            FlowerArm.DIRECTION = -FlowerArm.DIRECTION;
            message = "Direction flipped to " + (int) FlowerArm.DIRECTION + ". Press Y: the arm must go UP.";
        }
        if (gamepad1.startWasPressed()) save();

        // ---- jog or automatic ----
        double jog = -gamepad1.left_stick_y;
        if (Math.abs(jog) > 0.1) {
            cycling = false;
            arm.manual(jog);   // capped at the Up / Down power
        } else if (arm.getState() == FlowerArm.State.MANUAL) {
            arm.manual(0);
        } else {
            arm.update();
        }

        // ---- timing ----
        FlowerArm.State s = arm.getState();
        if (s != lastState) {
            if (s == FlowerArm.State.UP && lastState == FlowerArm.State.MOVING_UP) {
                lastUp = arm.getLastMoveSec();
                slowestUp = Math.max(slowestUp, lastUp);
                nextCycle();
            } else if (s == FlowerArm.State.DOWN && lastState == FlowerArm.State.MOVING_DOWN) {
                lastDown = arm.getLastMoveSec();
                slowestDown = Math.max(slowestDown, lastDown);
                nextCycle();
            } else if (s == FlowerArm.State.FAULT) {
                cycling = false;
            }
            lastState = s;
        }
        if (cycling && cycleMoves > 0 && pause.seconds() > 0.5
                && (s == FlowerArm.State.UP || s == FlowerArm.State.DOWN)) {
            if (s == FlowerArm.State.UP) arm.down(); else arm.up();
            pause.reset();
        }

        // ---- screen ----
        telemetry.addLine("=== FLOWER ARM TEST ===");
        telemetry.addData("Arm", arm.getStatus());
        telemetry.addData("Switches", arm.switchText());
        telemetry.addLine("Y up  A down  B stop  X cycle test  stick = jog  Back = flip direction  Start = SAVE");
        telemetry.addData("Up power / Down power", "%.2f / %.2f  (Dpad Up-Down / Right-Left, max %.2f)", FlowerArm.UP_POWER,
                FlowerArm.DOWN_POWER, FlowerArm.MAX_SAFE_POWER);
        telemetry.addData("Direction", "%+d", (int) FlowerArm.DIRECTION);
        telemetry.addData("Last move up / down (s)", "%s / %s", fmt(lastUp), fmt(lastDown));
        if (cycleMoves > 0) telemetry.addData("Cycle test", "%d moves, slowest up %.2f s, down %.2f s",
                cycleMoves, slowestUp, slowestDown);
        telemetry.addData("Will save", "auto waits up %.1f s / down %.1f s (arm limit %.0f s)",
                 suggestedWait(slowestUp, lastUp), suggestedWait(slowestDown, lastDown), FlowerArm.MAX_TRAVEL_SEC);
        if (!message.isEmpty()) telemetry.addLine(message);
        telemetry.update();
    }

    private void nextCycle() {
        if (!cycling) return;
        cycleMoves++;
        pause.reset();
    }

    private double suggestedMaxTravel() {
        double slow = Math.max(Math.max(slowestUp, slowestDown), Math.max(nz(lastUp), nz(lastDown)));
        return slow > 0 ? Math.ceil((slow * 1.5 + 0.3) * 10) / 10 : FlowerArm.MAX_TRAVEL_SEC;
    }

    private static double suggestedWait(double slowest, double last) {
        double slow = Math.max(slowest, nz(last));
        return slow > 0 ? Math.ceil((slow + 0.5) * 10) / 10 : Double.NaN;
    }

    private void save() {
        double up = suggestedWait(slowestUp, lastUp), down = suggestedWait(slowestDown, lastDown);
        if (!Double.isNaN(up)) AutoConfig.FLOWER_ARM_UP_SEC = up;
        if (!Double.isNaN(down)) AutoConfig.FLOWER_ARM_DOWN_SEC = down;
        message = RobotSettings.save()
                ? String.format(Locale.US, "SAVED: powers %.2f/%.2f, direction %+d, max travel %.1f s",
                        FlowerArm.UP_POWER, FlowerArm.DOWN_POWER, (int) FlowerArm.DIRECTION, FlowerArm.MAX_TRAVEL_SEC)
                : "SAVE FAILED: " + RobotSettings.problem();
    }

    private static double clamp(double v) { return Math.max(0.05, Math.min(FlowerArm.MAX_SAFE_POWER, Math.round(v * 100) / 100.0)); }

    private static double nz(double v) { return Double.isNaN(v) ? 0 : v; }

    private static String fmt(double v) { return Double.isNaN(v) ? "-" : String.format(Locale.US, "%.2f", v); }

    @Override
    public void stop() { arm.stop(); }
}
