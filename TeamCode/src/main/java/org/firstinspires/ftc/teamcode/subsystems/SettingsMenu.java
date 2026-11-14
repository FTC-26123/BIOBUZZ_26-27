package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.RobotSettings.Setting;

import java.util.List;
import java.util.Locale;

/**
 * The on-screen settings editor, shared by "Robot Settings" and "Tune Everything". Every change is saved at once.
 *
 * <p>Buttons (on whichever gamepad is passed in): LB / RB = group, Dpad Up / Down = pick (hold to scroll),
 * Dpad Right / Left = change (hold to repeat; ON/OFF toggles), A = steps x10, B = steps /10, X = code default.
 * Edge detection is done here from the raw buttons, so it does not use up the gamepad's "WasPressed" events.
 */
public final class SettingsMenu {
    private static final double REPEAT_DELAY = 0.40, REPEAT_EVERY = 0.07;

    private final int visibleRows;
    private int group = 0, row = 0;
    private double stepMult = 1;
    private String message = "";

    private final boolean[] held = new boolean[9];
    private final double[] next = new double[4];
    private final ElapsedTime[] timers = {new ElapsedTime(), new ElapsedTime(), new ElapsedTime(), new ElapsedTime()};

    public SettingsMenu(int visibleRows) {
        this.visibleRows = Math.max(3, visibleRows);
        RobotSettings.ensureLoaded();
    }

    /** Reads the buttons. @return the setting changed this call (already saved), or null. */
    public Setting handle(Gamepad g) {
        List<String> groups = RobotSettings.groups();
        if (pressed(g.right_bumper, 4)) { group = (group + 1) % groups.size(); row = 0; }
        if (pressed(g.left_bumper, 5)) { group = (group + groups.size() - 1) % groups.size(); row = 0; }
        List<Setting> list = RobotSettings.inGroup(groups.get(group));
        row = Math.max(0, Math.min(list.size() - 1, row));

        if (repeat(g.dpad_down, 0)) row = (row + 1) % list.size();
        if (repeat(g.dpad_up, 1)) row = (row + list.size() - 1) % list.size();
        if (pressed(g.a, 6)) stepMult = Math.min(1000, stepMult * 10);
        if (pressed(g.b, 7)) stepMult = Math.max(0.001, stepMult / 10);

        Setting s = list.get(row);
        boolean plus = repeat(g.dpad_right, 2), minus = repeat(g.dpad_left, 3);
        if (plus || minus) {
            double before = s.get();
            if (s.kind == RobotSettings.Kind.BOOL) s.set(before >= 0.5 ? 0 : 1);
            else s.set(before + (plus ? step(s) : -step(s)));
            saved(s.label + ": " + s.format(before) + " -> " + s.text());
            return s;
        }
        if (pressed(g.x, 8)) {
            s.set(s.codeDefault());
            saved(s.label + " = code default " + s.text());
            return s;
        }
        return null;
    }

    /** The setting under the cursor. */
    public Setting selected() {
        List<Setting> list = RobotSettings.inGroup(RobotSettings.groups().get(group));
        return list.get(Math.max(0, Math.min(list.size() - 1, row)));
    }

    public String groupName() { return RobotSettings.groups().get(group); }

    /** Group line, the visible rows, the help line and the step line. */
    public void show(Telemetry t) {
        List<String> groups = RobotSettings.groups();
        List<Setting> list = RobotSettings.inGroup(groups.get(group));
        t.addLine(String.format(Locale.US, "<< LB  %s (%d/%d)  RB >>", groups.get(group), group + 1, groups.size()));
        int first = Math.max(0, Math.min(row - visibleRows / 2, list.size() - visibleRows));
        for (int i = first; i < Math.min(list.size(), first + visibleRows); i++) {
            Setting x = list.get(i);
            String more = i == first && first > 0 ? "  ^" : i == first + visibleRows - 1 && i < list.size() - 1 ? "  v" : "";
            t.addLine(String.format(Locale.US, "%s %s = %s%s%s", i == row ? ">>" : "   ", x.label, x.text(),
                    x.isDefault() ? "" : " *", more));
        }
        Setting s = selected();
        t.addLine(s.help);
        t.addLine(s.kind == RobotSettings.Kind.BOOL
                ? "Dpad L/R toggles. Default " + s.format(s.codeDefault()) + " (X)"
                : "Step " + RobotSettings.trim(step(s)) + " (A x10, B /10). Default " + s.format(s.codeDefault()) + " (X)");
    }

    public String message() { return message; }

    public void setMessage(String m) { message = m; }

    private double step(Setting s) {
        return s.kind == RobotSettings.Kind.INT ? Math.max(1, Math.round(s.step * stepMult)) : s.step * stepMult;
    }

    private void saved(String what) {
        message = RobotSettings.save() ? "Saved. " + what : RobotSettings.problem();
    }

    private boolean pressed(boolean down, int i) {
        boolean was = held[i];
        held[i] = down;
        return down && !was;
    }

    /** true on the press, then repeatedly while held. */
    private boolean repeat(boolean down, int i) {
        boolean was = held[i];
        held[i] = down;
        if (!down) return false;
        if (!was) {
            timers[i].reset();
            next[i] = REPEAT_DELAY;
            return true;
        }
        if (timers[i].seconds() >= next[i]) {
            next[i] += REPEAT_EVERY;
            return true;
        }
        return false;
    }
}
