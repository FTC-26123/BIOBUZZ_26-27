package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Turns on the SDK's AUTO bulk-read mode on every Control/Expansion Hub. Encoder positions, velocities and
 * digital inputs then come from one bulk read per hub instead of one hub round trip each, which makes every loop
 * faster (better position hold, faster velocity checks).
 *
 * <p>AUTO mode needs no per-loop call: the cache refreshes automatically the next time any value is read a second
 * time. Motor current (getCurrent) and I2C devices such as the Pinpoint are not part of bulk data and are unaffected.
 * Safe to call more than once; every subsystem init calls it so each OpMode gets it without extra code.
 *
 * <p>Uses reflection on purpose: the desktop simulator and check scripts compile this file without the REV hub
 * classes, and there it simply does nothing.
 */
public final class BulkReads {
    private BulkReads() {}

    @SuppressWarnings({"unchecked", "rawtypes"})
    public static void enable(HardwareMap hardwareMap) {
        try {
            Class<?> lynx = Class.forName("com.qualcomm.hardware.lynx.LynxModule");
            Class modeClass = Class.forName("com.qualcomm.hardware.lynx.LynxModule$BulkCachingMode");
            Object auto = Enum.valueOf(modeClass, "AUTO");
            Iterable<?> hubs = (Iterable<?>) hardwareMap.getClass().getMethod("getAll", Class.class)
                    .invoke(hardwareMap, lynx);
            for (Object hub : hubs) {
                lynx.getMethod("setBulkCachingMode", modeClass).invoke(hub, auto);
            }
        } catch (Throwable ignored) {
            // No REV hubs (simulator / test fixture): reads simply stay uncached.
        }
    }
}
