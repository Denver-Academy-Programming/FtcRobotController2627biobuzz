package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Two-motor launcher, one motor per side, each with its own velocity loop so both sides match exactly
 * (a mismatch puts side-spin on the POLLEN and curves it).
 *
 * Per side:   power = (kS + kV * target) * 12V / battery   + kP * error
 *             full power when more than FLY_RECOVERY_BAND below target (fastest possible recovery after a shot)
 * Feedforward with battery compensation does almost all the work, so the speed is the same at 13.5 V and 12.2 V.
 *
 * Shot detection: a sudden drop in speed while feeding means a ball just went through. That lets the shooter fire
 * exactly one ball at a time, each at full speed.
 */
public class Flywheel {
    private final DcMotorEx left, right;

    private double target = 0;
    private double velL = 0, velR = 0;
    private double powerL = 0, powerR = 0;
    private double stableFor = 0;
    private int shots = 0;
    private boolean wasAtSpeed = false;

    // Recovery-time measurement (for tuning): time from a detected shot until ready again.
    private double sinceShot = -1;
    private double lastRecoverySec = Double.NaN;

    public Flywheel(HardwareMap hw) {
        left = Hw.opt(hw, DcMotorEx.class, "launcher_left");
        right = Hw.opt(hw, DcMotorEx.class, "launcher_right");
        if (left != null) left.setDirection(RobotConfig.LAUNCHER_LEFT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (right != null) right.setDirection(RobotConfig.LAUNCHER_RIGHT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            if (m == null) continue;
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // our own loop; encoder still reads
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast, don't brake a heavy flywheel
        }
    }

    public void setTarget(double ticksPerSec) {
        double t = Math.max(0, Math.min(RobotConfig.LAUNCHER_MAX_TICKS_PER_SEC, ticksPerSec));
        // A target step isn't a shot: re-arm shot detection only once we're back at speed.
        if (Math.abs(t - target) > RobotConfig.FLY_READY_TOLERANCE) wasAtSpeed = false;
        target = t;
    }

    /** Call every loop (after bulk cache clear). */
    public void update(double dt, double batteryVolts) {
        double a = RobotConfig.FLY_VELOCITY_FILTER;
        // Only one side configured: use it for both, so a single-motor launcher still works.
        DcMotorEx l = left != null ? left : right, r = right != null ? right : left;
        if (l == null) { velL = velR = 0; stableFor = 0; return; }   // no launcher at all
        velL += a * (l.getVelocity() - velL);
        velR += a * (r.getVelocity() - velR);

        if (target <= 0) {
            powerL = powerR = 0;
        } else {
            powerL = sidePower(velL, batteryVolts);
            powerR = sidePower(velR, batteryVolts);
        }
        if (left != null) left.setPower(powerL);
        if (right != null) right.setPower(powerR);

        // Ready / shot detection
        boolean inBand = target > 0
                && Math.abs(target - velL) <= RobotConfig.FLY_READY_TOLERANCE
                && Math.abs(target - velR) <= RobotConfig.FLY_READY_TOLERANCE
                && Math.abs(velL - velR) <= RobotConfig.FLY_MAX_SIDE_MISMATCH;
        stableFor = inBand ? stableFor + dt : 0;

        double slowest = Math.min(velL, velR);
        if (wasAtSpeed && target > 0 && target - slowest > RobotConfig.FLY_SHOT_DROP) {
            shots++;
            sinceShot = 0;
            wasAtSpeed = false;
        }
        if (stableFor > 0) wasAtSpeed = true;

        if (sinceShot >= 0) {
            sinceShot += dt;
            if (ready()) { lastRecoverySec = sinceShot; sinceShot = -1; }
        }
    }

    private double sidePower(double vel, double volts) {
        double err = target - vel;
        if (err > RobotConfig.FLY_RECOVERY_BAND) return 1.0;
        double ff = (RobotConfig.FLY_KS + RobotConfig.FLY_KV * target) * 12.0 / Math.max(volts, 9.0);
        return Math.max(0, Math.min(1, ff + RobotConfig.FLY_KP * err));
    }

    public boolean present() { return left != null || right != null; }

    public boolean ready() { return stableFor >= RobotConfig.FLY_READY_STABLE_SEC; }

    /** Total balls detected leaving the launcher. */
    public int shotCount() { return shots; }

    public double target() { return target; }
    public double velocityLeft() { return velL; }
    public double velocityRight() { return velR; }
    public double powerLeft() { return powerL; }
    public double powerRight() { return powerR; }
    public double lastRecoverySec() { return lastRecoverySec; }

    public void stop() {
        target = 0;
        if (left != null) left.setPower(0);
        if (right != null) right.setPower(0);
    }
}
