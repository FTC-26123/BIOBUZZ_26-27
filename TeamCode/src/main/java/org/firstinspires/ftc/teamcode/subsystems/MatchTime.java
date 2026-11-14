package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Match lengths and the time warnings (Robot Settings > Match).
 * <ul>
 *   <li>The autonomous program stops itself at {@link #AUTO_SEC}, TeleOp at {@link #TELEOP_SEC}, counted from Play.
 *       (In a real match the Driver Station also stops them; this is the backup, and it ends practice runs on time.)</li>
 *   <li>LED warnings: {@link #FIRST_WARNING_LEFT_SEC} left = slow WHITE pulse, {@link #LAST_WARNING_LEFT_SEC} left =
 *       fast WHITE strobe, each for {@link #WARNING_FLASH_SEC} seconds. Both in autonomous and TeleOp.</li>
 * </ul>
 */
public final class MatchTime {
    private MatchTime() { }

    /** Autonomous length, seconds from Play. The program stops itself here. */
    public static double AUTO_SEC = 30;
    /** TeleOp length, seconds from Play. The program stops itself here. 0 = never (long practice sessions). */
    public static double TELEOP_SEC = 120;
    /** Seconds left at the first warning (slow white pulse). TeleOp only: autonomous is too short. <0 = off. */
    public static double FIRST_WARNING_LEFT_SEC = 30;
    /** Seconds left at the last warning (fast white strobe), autonomous and TeleOp. <0 = off. */
    public static double LAST_WARNING_LEFT_SEC = 10;
    /** How long each warning flashes. */
    public static double WARNING_FLASH_SEC = 2.0;

    /** @return true once the program has run its full length (length <= 0 = never). */
    public static boolean over(double elapsedSec, double lengthSec) {
        return lengthSec > 0 && elapsedSec >= lengthSec;
    }

    /** @return 2 = last warning flashing now, 1 = first warning flashing now, 0 = none. */
    public static int warning(double elapsedSec, double lengthSec, boolean firstWarningToo) {
        if (lengthSec <= 0) return 0;
        double left = lengthSec - elapsedSec;
        if (LAST_WARNING_LEFT_SEC >= 0 && left <= LAST_WARNING_LEFT_SEC && left > LAST_WARNING_LEFT_SEC - WARNING_FLASH_SEC)
            return 2;
        if (firstWarningToo && FIRST_WARNING_LEFT_SEC >= 0 && left <= FIRST_WARNING_LEFT_SEC
                && left > FIRST_WARNING_LEFT_SEC - WARNING_FLASH_SEC)
            return 1;
        return 0;
    }
}
