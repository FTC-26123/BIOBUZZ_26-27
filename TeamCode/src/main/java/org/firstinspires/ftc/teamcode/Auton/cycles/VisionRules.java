package org.firstinspires.ftc.teamcode.Auton.cycles;

/** Shared sensor contract, independent of camera SDKs for regression checks. */
public final class VisionRules {
    public static final int POLLEN_PIPELINE = 7;
    public static final String POLLEN_LABEL = "yellow_pollen";
    public static final double MAX_AGE_MS = 180;
    private VisionRules() { }
    public static boolean pollen(String label, double confidence, double tx, double ty) {
        return POLLEN_LABEL.equals(label) && Double.isFinite(confidence) && confidence > 0
                && Double.isFinite(tx) && Double.isFinite(ty);
    }
    public static boolean freshTag(long now, long acquired) {
        return acquired > 0 && acquired <= now && now - acquired <= (long)(MAX_AGE_MS * 1000000);
    }
    public static boolean expectedTag(int id, int first) { return id >= first && id < first + 4; }
}
