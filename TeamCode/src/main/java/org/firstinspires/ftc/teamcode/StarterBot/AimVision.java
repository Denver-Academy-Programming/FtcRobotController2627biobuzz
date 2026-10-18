package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.concurrent.TimeUnit;

/*
 * Finds our alliance's upward-facing HIVE CELL by its AprilTags and turns it into what the shooter needs:
 *   - horizontal distance from the launcher to the CELL (for Ballistics.solve)
 *   - heading error: how far the robot has to turn so the launcher points at the CELL
 *
 * BIOBUZZ AprilTags (Competition Manual TU04, section 9.9, Figure 9-17):
 *   36h11 family, 3.25 in square, 4 tags per CELL, stuck on the BOTTOM of each CELL facing DOWN at the tiles.
 *   Red  CELL, far side (opposite audience): 30 31 32 33     Red  CELL, audience side: 34 35 36 37
 *   Blue CELL, audience side:                38 39 40 41     Blue CELL, far side:      42 43 44 45
 * The tags move every time a HIVE tips, so they're only good for aiming, not for field localization.
 *
 * Because the tags face down, the webcam MUST be tilted up to see them. Set CAMERA_PITCH_UP_DEG and
 * CAMERA_HEIGHT_IN so the measured tag heights on telemetry look right.
 *
 * Each HIVE has two CELLS and only the one facing up can be scored in. Its tags sit higher than the
 * lowered CELL's tags (the lowered CELL's bottom is 30.6 in above the tiles, Figure 9-10), so we pick
 * the highest alliance tag and ignore anything below UP_CELL_MIN_TAG_HEIGHT_IN.
 *
 * Assumes the webcam faces the same way as the launcher (front of the robot).
 * Robot configuration name: "Webcam 1".
 */
public class AimVision {

    public static final int[] RED_HIVE_TAG_IDS = {30, 31, 32, 33, 34, 35, 36, 37};
    public static final int[] BLUE_HIVE_TAG_IDS = {38, 39, 40, 41, 42, 43, 44, 45};

    // ----- MEASURE THESE ON YOUR ROBOT -----
    // Camera tilt above horizontal, and lens height above the tiles.
    public static final double CAMERA_PITCH_UP_DEG = 30.0;
    public static final double CAMERA_HEIGHT_IN = 10.0;

    // Where the camera lens is relative to the launcher exit, looking from above (inches).
    // +right / +forward. e.g. camera 3" to the right of the launcher and 2" in front of it -> 3, 2.
    public static final double CAMERA_RIGHT_OF_LAUNCHER_IN = 0.0;
    public static final double CAMERA_FORWARD_OF_LAUNCHER_IN = 0.0;

    // ----- TUNE ON THE FIELD -----
    // Tags lower than this belong to the lowered CELL (can't score). Estimated from Figure 9-10:
    // lowered CELL bottom is 30.6 in, raised CELL opening starts at 53.5 in. Check "tag height" on telemetry
    // with each CELL up and set this halfway between the two readings.
    public static final double UP_CELL_MIN_TAG_HEIGHT_IN = 42.0;

    // Horizontal distance from the tags to the middle of the CELL opening, along the line of sight.
    // + = opening is farther away than the tags. Adjust if shots are consistently long/short at every distance
    // after efficiency is calibrated.
    public static final double TARGET_BEHIND_TAG_IN = 0.0;

    // Constant heading trim in degrees if shots consistently go left (-) or right (+) once aimed.
    public static final double AIM_TRIM_DEG = 0.0;

    // Keep using the last detection this long after the tag drops out (motion blur, someone drives past).
    private static final double HOLD_LAST_TARGET_SEC = 0.25;

    // Short exposure reduces motion blur so tags are still found while turning.
    private static final int EXPOSURE_MS = 6;
    private static final int GAIN = 250;

    private VisionPortal portal;
    private AprilTagProcessor aprilTag;
    private boolean exposureSet = false;
    private boolean redAlliance = true;

    private final ElapsedTime sinceSeen = new ElapsedTime();
    private boolean haveTarget = false;
    private int targetId = -1;
    private double distanceIn, headingErrorDeg, tagHeightIn;
    private String seenIds = "";

    public void init(HardwareMap hardwareMap) {
        // The SDK's default tag library is the current season's, so BIOBUZZ tags 30-45 come with their size.
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2); // ~22 fps. Lower = detects farther but slower.

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public void setRedAlliance(boolean red) { redAlliance = red; }

    public boolean isRedAlliance() { return redAlliance; }

    /** Call once per loop. */
    public void update() {
        trySetExposure();

        double pitch = Math.toRadians(CAMERA_PITCH_UP_DEG);
        boolean found = false;
        int bestId = -1;
        double bestRight = 0, bestForward = 0, bestHeight = -Double.MAX_VALUE;
        StringBuilder ids = new StringBuilder();

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (!(detection instanceof AprilTagSingleDetection)) continue;
            AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
            ids.append(single.id).append(single.metadata == null ? "? " : " "); // "?" = not in library, no pose
            if (single.metadata == null || !isAllianceTag(single.id)) continue;

            // Camera frame: +x right, +y out of the lens, +z up out of the top of the camera.
            // Undo the camera's upward tilt to get robot-level right / forward / up.
            double x = single.ftcPose.x, y = single.ftcPose.y, z = single.ftcPose.z;
            double forward = y * Math.cos(pitch) - z * Math.sin(pitch);
            double up = y * Math.sin(pitch) + z * Math.cos(pitch);
            double height = CAMERA_HEIGHT_IN + up;

            if (height >= UP_CELL_MIN_TAG_HEIGHT_IN && height > bestHeight) {
                found = true;
                bestId = single.id;
                bestRight = x;
                bestForward = forward;
                bestHeight = height;
            }
        }
        seenIds = ids.toString().trim();

        if (found) {
            // Push the target point back from the tags along the line of sight, then shift from camera to launcher.
            double flat = Math.max(Math.hypot(bestRight, bestForward), 1e-6);
            double scale = (flat + TARGET_BEHIND_TAG_IN) / flat;
            double right = bestRight * scale + CAMERA_RIGHT_OF_LAUNCHER_IN;
            double forward = bestForward * scale + CAMERA_FORWARD_OF_LAUNCHER_IN;

            distanceIn = Math.hypot(right, forward);
            headingErrorDeg = Math.toDegrees(Math.atan2(right, forward)) + AIM_TRIM_DEG; // + = target is to the right
            tagHeightIn = bestHeight;
            targetId = bestId;
            haveTarget = true;
            sinceSeen.reset();
        } else if (sinceSeen.seconds() > HOLD_LAST_TARGET_SEC) {
            haveTarget = false;
        }
    }

    public boolean hasTarget() { return haveTarget; }

    /** Horizontal inches from launcher exit to the CELL target point. */
    public double distanceIn() { return distanceIn; }

    /** Degrees the robot must turn clockwise to point the launcher at the target. */
    public double headingErrorDeg() { return headingErrorDeg; }

    /** Measured height of the tag being used, inches above the tiles. Use to check camera pitch/height. */
    public double tagHeightIn() { return tagHeightIn; }

    public String targetName() {
        if (!haveTarget) return "none";
        String side = (targetId <= 33 || targetId >= 42) ? "far" : "audience";
        return String.format("%s %s CELL (tag %d)", redAlliance ? "RED" : "BLUE", side, targetId);
    }

    /** Every tag ID currently in view. */
    public String seenIds() { return seenIds.isEmpty() ? "-" : seenIds; }

    public void close() {
        if (portal != null) portal.close();
    }

    private boolean isAllianceTag(int id) {
        for (int hiveId : redAlliance ? RED_HIVE_TAG_IDS : BLUE_HIVE_TAG_IDS) if (hiveId == id) return true;
        return false;
    }

    // Exposure controls only work once the camera is streaming, so keep trying until it is.
    private void trySetExposure() {
        if (exposureSet || portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        if (exposure.getMode() != ExposureControl.Mode.Manual) exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
        portal.getCameraControl(GainControl.class).setGain(GAIN);
        exposureSet = true;
    }
}
