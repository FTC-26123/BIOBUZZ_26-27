package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Tells the code whether it is running inside the BIOBUZZ desktop simulator instead of on the robot.
 *
 * <p>Used only to switch on the optional features (camera aiming, flower arm) in the simulator so they can be
 * tried there before their real-robot switches are turned on. On the real robot {@link #YES} is always false.
 */
public final class InSimulator {
    private InSimulator() { }

    /** true in the simulator, false on the robot. */
    public static final boolean YES = detect();

    private static boolean detect() {
        try {
            // This class exists only in the simulator, never in the FTC app on the robot.
            // (Looked up without starting it, so this is safe at any time.)
            Class.forName("virtual_robot.biobuzz.Settings", false, InSimulator.class.getClassLoader());
            return true;
        } catch (Throwable notTheSimulator) {
            return false;
        }
    }
}
