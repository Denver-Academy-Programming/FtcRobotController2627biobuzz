package org.firstinspires.ftc.teamcode.biobuzz;

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
 * Locates our alliance's raised HIVE CELL on the FIELD (odometry frame) from its AprilTags.
 *
 * BIOBUZZ tags (Competition Manual TU04 9.9, Fig 9-17): 36h11, 4 per CELL, on the BOTTOM of each CELL facing down.
 *   Red  far CELL 30-33, Red  audience CELL 34-37, Blue audience CELL 38-41, Blue far CELL 42-45.
 * Only the CELL facing up scores; its tags sit higher, so we pick the alliance CELL with the highest tags.
 *
 * Why field coordinates instead of "aim at what the camera sees":
 *   - Each sighting is placed on the field using the robot pose at the moment the frame was captured
 *     (frameAcquisitionNanoTime + Drivetrain pose history), so camera latency and turning don't smear it.
 *   - Sightings are averaged (all visible tags of the CELL, and over time), which beats any single noisy tag.
 *   - Between sightings (blur, someone drives past, launcher turned away) we keep aiming with odometry + IMU,
 *     which updates every loop instead of every camera frame.
 * When the HIVE tips the target jumps, which resets the estimate.
 */
public class HiveVision {

    private VisionPortal portal;
    private AprilTagProcessor aprilTag;
    private boolean exposureSet = false;
    private boolean redAlliance = true;

    private boolean haveTarget = false;
    private boolean seeded = false;          // target came from the auto plan, not from a sighting
    private double tx, ty;                   // field position of the CELL center
    private int targetCell = -1;
    private final ElapsedTime sinceSeen = new ElapsedTime();

    // Diagnostics
    private String seenIds = "";
    private double lastTagHeight = Double.NaN;
    private int lastTagCount = 0;

    public void init(HardwareMap hw) {
        aprilTag = new AprilTagProcessor.Builder().build(); // default library = this season's tags with sizes
        aprilTag.setDecimation(2);
        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public void setRedAlliance(boolean red) {
        if (red != redAlliance) clearTarget();
        redAlliance = red;
    }

    public boolean isRedAlliance() { return redAlliance; }

    /** Give a starting guess (field coordinates), e.g. from the autonomous plan. Real sightings replace it. */
    public void seedTarget(double fieldX, double fieldY) {
        tx = fieldX;
        ty = fieldY;
        haveTarget = true;
        seeded = true;
        targetCell = -1;
    }

    public void clearTarget() {
        haveTarget = false;
        seeded = false;
        targetCell = -1;
    }

    /** Call once per loop, after Drivetrain.update(). */
    public void update(Drivetrain drive) {
        trySetExposure();

        double pitch = Math.toRadians(RobotConfig.CAMERA_PITCH_UP_DEG);
        double yaw = Math.toRadians(RobotConfig.CAMERA_YAW_DEG);
        double cy = Math.cos(yaw), sy = Math.sin(yaw);

        // Per CELL: sum of field x/y, sum of heights, count.
        double[] sumX = new double[4], sumY = new double[4], sumH = new double[4];
        int[] count = new int[4];
        StringBuilder ids = new StringBuilder();

        for (AprilTagDetection det : aprilTag.getDetections()) {
            if (!(det instanceof AprilTagSingleDetection)) continue;
            AprilTagSingleDetection tag = (AprilTagSingleDetection) det;
            ids.append(tag.id).append(tag.metadata == null ? "? " : " ");
            if (tag.metadata == null || tag.ftcPose == null) continue;
            int cell = cellOf(tag.id);
            if (cell < 0 || isRedCell(cell) != redAlliance) continue;

            // Camera frame: +x right, +y out of the lens, +z up. Level it, then rotate/shift into the robot frame.
            double camX = tag.ftcPose.x, camY = tag.ftcPose.y, camZ = tag.ftcPose.z;
            double fwd = camY * Math.cos(pitch) - camZ * Math.sin(pitch);
            double up = camY * Math.sin(pitch) + camZ * Math.cos(pitch);
            double left = -camX;
            double robotF = RobotConfig.CAMERA_FORWARD_IN + fwd * cy - left * sy;
            double robotL = RobotConfig.CAMERA_LEFT_IN + fwd * sy + left * cy;

            Pose then = drive.poseAt(det.frameAcquisitionNanoTime);
            double[] field = then.toField(robotF, robotL);
            sumX[cell] += field[0];
            sumY[cell] += field[1];
            sumH[cell] += RobotConfig.CAMERA_HEIGHT_IN + up;
            count[cell]++;
        }
        seenIds = ids.toString().trim();

        // Highest alliance CELL above the threshold is the raised one.
        int best = -1;
        double bestH = -Double.MAX_VALUE;
        for (int c = 0; c < 4; c++) {
            if (count[c] == 0) continue;
            double h = sumH[c] / count[c];
            if (h >= RobotConfig.UP_CELL_MIN_TAG_HEIGHT_IN && h > bestH) { bestH = h; best = c; }
        }
        if (best < 0) return;

        double mx = sumX[best] / count[best], my = sumY[best] / count[best];
        lastTagHeight = bestH;
        lastTagCount = count[best];

        boolean jump = haveTarget && Math.hypot(mx - tx, my - ty) > RobotConfig.TARGET_RESET_JUMP_IN;
        if (!haveTarget || seeded || jump || best != targetCell) {
            tx = mx;
            ty = my;
        } else {
            // More tags in view = more trustworthy sighting.
            double a = Math.min(1.0, RobotConfig.TARGET_FILTER_ALPHA * (0.5 + 0.5 * count[best]));
            tx += a * (mx - tx);
            ty += a * (my - ty);
        }
        targetCell = best;
        haveTarget = true;
        seeded = false;
        sinceSeen.reset();
    }

    /** True if we know where the target is (recently seen, or seeded by auto). */
    public boolean hasTarget() {
        if (haveTarget && !seeded && sinceSeen.seconds() > RobotConfig.TARGET_MEMORY_SEC) haveTarget = false;
        return haveTarget;
    }

    /** True if a tag was seen in the last few frames (not just remembered). */
    public boolean seesTarget() {
        return haveTarget && !seeded && sinceSeen.seconds() < 0.15;
    }

    public double targetX() { return tx; }
    public double targetY() { return ty; }

    public String targetName() {
        if (!hasTarget()) return "none";
        if (seeded) return "planned position (no tag yet)";
        String[] names = {"RED far", "RED audience", "BLUE audience", "BLUE far"};
        return String.format("%s CELL, %d tags, %.1fs ago", targetCell >= 0 ? names[targetCell] : "?",
                lastTagCount, sinceSeen.seconds());
    }

    public String seenIds() { return seenIds.isEmpty() ? "-" : seenIds; }

    /** Measured height of the chosen CELL's tags. Use it to check CAMERA_PITCH / HEIGHT and UP_CELL_MIN_TAG_HEIGHT. */
    public double lastTagHeight() { return lastTagHeight; }

    public void close() {
        if (portal != null) portal.close();
    }

    private static int cellOf(int id) {
        return (id >= 30 && id <= 45) ? (id - 30) / 4 : -1;
    }

    private static boolean isRedCell(int cell) {
        return cell <= 1;
    }

    // Exposure controls only work once the camera is streaming.
    private void trySetExposure() {
        if (exposureSet || portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        if (exposure.getMode() != ExposureControl.Mode.Manual) exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(RobotConfig.CAMERA_EXPOSURE_MS, TimeUnit.MILLISECONDS);
        portal.getCameraControl(GainControl.class).setGain(RobotConfig.CAMERA_GAIN);
        exposureSet = true;
    }
}
