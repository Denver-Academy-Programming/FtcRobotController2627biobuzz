package org.firstinspires.ftc.teamcode.FullBot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/*
 * Optional hardware lookup. A device that isn't in the Driver Hub configuration returns null instead of crashing
 * the OpMode, and its name is remembered so telemetry can show what's missing. Every subsystem treats null as
 * "not wired yet" and skips it.
 *
 * Note: a device that IS in the configuration but not physically plugged in is found fine; it just reads 0.
 */
public final class Hw {
    private static final List<String> missing = new ArrayList<>();

    private Hw() {}

    /** Clear the missing list (call once when building a new Robot). */
    public static void reset() { missing.clear(); }

    public static <T> T opt(HardwareMap hw, Class<? extends T> type, String name) {
        T device = null;
        try {
            device = hw.tryGet(type, name);
        } catch (RuntimeException ignored) {
            // wrong device type in the config for this name: treat as missing
        }
        if (device == null) missing.add(name);
        return device;
    }

    /** Names of configured-in-code devices that weren't found, e.g. "intake, hood". Empty if all present. */
    public static String missing() {
        return String.join(", ", missing);
    }
}
