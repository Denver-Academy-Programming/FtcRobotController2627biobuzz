package org.firstinspires.ftc.teamcode.FullBot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Mecanum drive + odometry.
 *
 * Odometry: wheel encoders for distance, IMU for heading (the IMU is far better at heading than wheel slip math).
 * Each loop the robot-frame motion is rotated into the field frame using the mid-step heading.
 * A short pose history lets vision ask "where was the robot when this camera frame was taken?" so a moving
 * robot doesn't smear the target estimate by camera latency.
 *
 * Drive commands use the math convention: forward, left, turn CCW (all -1..1).
 */
public class Drivetrain {
    private final DcMotorEx fl, fr, rl, rr;
    private final DcMotorEx[] motors;
    private final IMU imu;

    private double x, y, heading;
    private double headingOffset;
    private double vx, vy, omega;                 // field frame, in/s and rad/s
    private int lastFl, lastFr, lastRl, lastRr;
    private double lastYaw;

    private double cmdForward, cmdLeft;           // slew-limited translation command (autonomous)

    // Pose history ring buffer for vision latency compensation.
    private static final int HISTORY = 64;
    private final long[] histT = new long[HISTORY];
    private final double[] histX = new double[HISTORY], histY = new double[HISTORY], histH = new double[HISTORY];
    private int histHead = 0, histCount = 0;

    public Drivetrain(HardwareMap hw) {
        fl = Hw.opt(hw, DcMotorEx.class, "Front_Left");
        fr = Hw.opt(hw, DcMotorEx.class, "Front_Right");
        rl = Hw.opt(hw, DcMotorEx.class, "Rear_Left");
        rr = Hw.opt(hw, DcMotorEx.class, "Rear_Right");
        motors = new DcMotorEx[]{fl, fr, rl, rr};

        DcMotorSimple.Direction leftDir = RobotConfig.LEFT_DRIVE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction rightDir = RobotConfig.RIGHT_DRIVE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        if (fl != null) fl.setDirection(leftDir);
        if (rl != null) rl.setDirection(leftDir);
        if (fr != null) fr.setDirection(rightDir);
        if (rr != null) rr.setDirection(rightDir);
        for (DcMotorEx m : motors) {
            if (m == null) continue;
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        imu = Hw.opt(hw, IMU.class, "imu");
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.valueOf(RobotConfig.HUB_LOGO_DIRECTION),
                    RevHubOrientationOnRobot.UsbFacingDirection.valueOf(RobotConfig.HUB_USB_DIRECTION))));
            imu.resetYaw();
        }
        setPose(new Pose(0, 0, 0));
    }

    /** Set where the robot is right now (start of auto, or a heading reset in TeleOp). */
    public void setPose(Pose p) {
        lastFl = pos(fl);
        lastFr = pos(fr);
        lastRl = pos(rl);
        lastRr = pos(rr);
        double yaw = yaw();
        headingOffset = p.heading - yaw;
        lastYaw = yaw;
        x = p.x;
        y = p.y;
        heading = Pose.wrap(p.heading);
        histCount = 0;
    }

    /** Call once per loop, after the bulk cache is cleared. */
    public void update() {
        int pFl = pos(fl), pFr = pos(fr);
        int pRl = pos(rl), pRr = pos(rr);
        double dFl = pFl - lastFl, dFr = pFr - lastFr, dRl = pRl - lastRl, dRr = pRr - lastRr;
        lastFl = pFl; lastFr = pFr; lastRl = pRl; lastRr = pRr;

        double yaw = yaw();
        double dYaw = Pose.wrap(yaw - lastYaw);
        lastYaw = yaw;

        // Robot-frame displacement. Turning cancels out of both sums.
        double dForward = (dFl + dFr + dRl + dRr) / 4.0 / RobotConfig.DRIVE_TICKS_PER_INCH;
        double dLeft = (-dFl + dFr + dRl - dRr) / 4.0 / RobotConfig.DRIVE_TICKS_PER_INCH * RobotConfig.STRAFE_MULTIPLIER;

        double mid = heading + dYaw / 2;
        x += dForward * Math.cos(mid) - dLeft * Math.sin(mid);
        y += dForward * Math.sin(mid) + dLeft * Math.cos(mid);
        heading = Pose.wrap(yaw + headingOffset);

        // Velocity straight from the hub's encoder velocity (bulk read) and the gyro.
        double vF = (vel(fl) + vel(fr) + vel(rl) + vel(rr)) / 4.0 / RobotConfig.DRIVE_TICKS_PER_INCH;
        double vL = (-vel(fl) + vel(fr) + vel(rl) - vel(rr)) / 4.0
                / RobotConfig.DRIVE_TICKS_PER_INCH * RobotConfig.STRAFE_MULTIPLIER;
        double c = Math.cos(heading), s = Math.sin(heading);
        vx = vF * c - vL * s;
        vy = vF * s + vL * c;
        omega = imu != null ? imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate : 0;

        histT[histHead] = System.nanoTime();
        histX[histHead] = x;
        histY[histHead] = y;
        histH[histHead] = heading;
        histHead = (histHead + 1) % HISTORY;
        histCount = Math.min(histCount + 1, HISTORY);
    }

    public Pose pose() { return new Pose(x, y, heading); }
    public double vx() { return vx; }
    public double vy() { return vy; }
    public double omega() { return omega; }

    /** Where the robot was at a past System.nanoTime() (interpolated). Falls back to the current pose. */
    public Pose poseAt(long nanoTime) {
        if (histCount == 0 || nanoTime <= 0) return pose();
        int newest = (histHead - 1 + HISTORY) % HISTORY;
        if (nanoTime >= histT[newest]) return pose();
        for (int i = 1; i < histCount; i++) {
            int a = (newest - i + HISTORY) % HISTORY;      // older
            int b = (a + 1) % HISTORY;                     // newer
            if (histT[a] <= nanoTime) {
                double w = (double) (nanoTime - histT[a]) / Math.max(1, histT[b] - histT[a]);
                return new Pose(histX[a] + w * (histX[b] - histX[a]),
                        histY[a] + w * (histY[b] - histY[a]),
                        histH[a] + w * Pose.wrap(histH[b] - histH[a]));
            }
        }
        int oldest = (newest - histCount + 1 + HISTORY) % HISTORY;
        return new Pose(histX[oldest], histY[oldest], histH[oldest]);
    }

    // ---------------- Driving ----------------

    /** Robot-centric. forward, left, turnCcw in -1..1. */
    public void drive(double forward, double left, double turnCcw) {
        double flP = forward - left - turnCcw;
        double frP = forward + left + turnCcw;
        double rlP = forward + left - turnCcw;
        double rrP = forward - left + turnCcw;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(flP), Math.abs(frP)), Math.max(Math.abs(rlP), Math.abs(rrP))));
        power(fl, flP / max);
        power(fr, frP / max);
        power(rl, rlP / max);
        power(rr, rrP / max);
    }

    // ---- Missing-hardware-safe access: an unconfigured motor reads 0 and ignores power; no IMU = heading stays put ----
    private static int pos(DcMotorEx m) { return m == null ? 0 : m.getCurrentPosition(); }
    private static double vel(DcMotorEx m) { return m == null ? 0 : m.getVelocity(); }
    private static void power(DcMotorEx m, double p) { if (m != null) m.setPower(p); }
    private double yaw() { return imu == null ? 0 : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); }
    public boolean hasImu() { return imu != null; }

    /** Field-centric: xField / yField are powers along the field axes. */
    public void driveField(double xField, double yField, double turnCcw) {
        double c = Math.cos(heading), s = Math.sin(heading);
        drive(xField * c + yField * s, -xField * s + yField * c, turnCcw);
    }

    /** Turn power (CCW) that holds / reaches a field heading. Uses the gyro rate as the D term. */
    public double headingPower(double targetHeading) {
        double err = Pose.wrap(targetHeading - heading);
        double p = RobotConfig.HEADING_KP * err - RobotConfig.HEADING_KD * omega;
        if (Math.abs(err) > Math.toRadians(0.3)) p += Math.copySign(RobotConfig.HEADING_KS, err);
        return Math.max(-RobotConfig.HEADING_MAX_POWER, Math.min(RobotConfig.HEADING_MAX_POWER, p));
    }

    public double headingError(double targetHeading) {
        return Pose.wrap(targetHeading - heading);
    }

    /**
     * One step of driving to a field pose. Returns true once within tolerance and nearly stopped.
     * maxPower limits translation speed. dt is the loop time (for the slew limit).
     */
    public boolean goTo(Pose target, double maxPower, double dt) {
        double ex = target.x - x, ey = target.y - y;
        double dist = Math.hypot(ex, ey);

        double speed = 0;
        if (dist > RobotConfig.POSITION_TOLERANCE_IN * 0.5) {
            speed = Math.min(maxPower, RobotConfig.TRANSLATE_KP * dist + RobotConfig.TRANSLATE_KS);
        }
        double c = Math.cos(heading), s = Math.sin(heading);
        double fwd = dist > 1e-6 ? speed * (ex * c + ey * s) / dist : 0;
        double left = dist > 1e-6 ? speed * (-ex * s + ey * c) / dist : 0;

        double maxStep = RobotConfig.TRANSLATE_MAX_ACCEL * dt;
        cmdForward += clamp(fwd - cmdForward, maxStep);
        cmdLeft += clamp(left - cmdLeft, maxStep);
        // Never slew while decelerating close to target (would overshoot).
        if (Math.abs(fwd) < Math.abs(cmdForward)) cmdForward = fwd;
        if (Math.abs(left) < Math.abs(cmdLeft)) cmdLeft = left;

        drive(cmdForward, cmdLeft, headingPower(target.heading));

        return dist < RobotConfig.POSITION_TOLERANCE_IN
                && Math.abs(headingError(target.heading)) < Math.toRadians(RobotConfig.HEADING_TOLERANCE_DEG)
                && Math.hypot(vx, vy) < 4;
    }

    public void stop() {
        cmdForward = cmdLeft = 0;
        drive(0, 0, 0);
    }

    private static double clamp(double v, double lim) {
        return Math.max(-lim, Math.min(lim, v));
    }
}
