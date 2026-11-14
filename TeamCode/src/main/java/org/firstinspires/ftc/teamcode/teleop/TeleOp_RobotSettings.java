package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlowerArm;
import org.firstinspires.ftc.teamcode.subsystems.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.RobotSettings.Setting;
import org.firstinspires.ftc.teamcode.subsystems.SettingsMenu;

import java.util.Locale;

/**
 * Edit every robot setting from the Driver Station. Nothing drives. Each change is saved on the Control Hub
 * straight away and used by the next OpMode you run: no code change, no re-upload.
 *
 * <p>Gamepad 1 (works in INIT too, no need to press Play):
 * <ul>
 *   <li>LB / RB = previous / next group</li>
 *   <li>Dpad Up / Down = pick a setting (hold to scroll)</li>
 *   <li>Dpad Right / Left = change it up / down (hold to repeat); ON/OFF settings toggle</li>
 *   <li>A = bigger steps (x10), B = smaller steps (/10)</li>
 *   <li>X = put this setting back to the code default</li>
 *   <li>Hold Back + Start for 2 s = put EVERY setting back to the code defaults</li>
 * </ul>
 * After Play, while 'Up position' or 'Down position' of the flower arm is selected, the arm servo moves to that
 * value as you change it, so the two positions can be found right here.
 */
@TeleOp(name = "Robot Settings (edit + save)", group = "Setup")
public class TeleOp_RobotSettings extends OpMode {

    private SettingsMenu menu;
    private final ElapsedTime resetAllTimer = new ElapsedTime();
    private boolean resetAllHeld, resetAllDone;
    private boolean playing = false;

    @Override
    public void init() {
        menu = new SettingsMenu(9);
        String p = RobotSettings.problem();
        menu.setMessage(p.isEmpty() ? "File: " + RobotSettings.file() : p);
    }

    @Override
    public void start() { playing = true; }

    @Override
    public void init_loop() { edit(); }

    @Override
    public void loop() { edit(); }

    private void edit() {
        menu.handle(gamepad1);
        Setting s = menu.selected();

        boolean armSetting = s.group.equals("Flower arm");

        // Reset everything: hold Back + Start for 2 seconds.
        boolean both = gamepad1.back && gamepad1.start;
        if (both && !resetAllHeld) { resetAllTimer.reset(); resetAllDone = false; }
        resetAllHeld = both;
        if (both && !resetAllDone) {
            if (resetAllTimer.seconds() >= 2.0) {
                resetAllDone = true;
                menu.setMessage(RobotSettings.resetAllToCode() ? "ALL settings back to code defaults (saved)"
                        : RobotSettings.problem());
            } else {
                menu.setMessage(String.format(Locale.US, "Keep holding to reset EVERYTHING... %.1f",
                        2.0 - resetAllTimer.seconds()));
            }
        }

        menu.show(telemetry);
        if (armSetting) {
            telemetry.addLine("Test and tune the arm with the \"Flower Arm Test\" OpMode.");
        }
        telemetry.addLine("* = changed from code.  " + menu.message());
        telemetry.update();
    }
}
