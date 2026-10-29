package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Launch-angle servo. Converts degrees to servo position using two measured calibration points, and estimates
 * where the servo actually is (servos have no feedback) so we never fire while the hood is still moving.
 */
public class Hood {
    private final Servo servo;
    private double commandedPos = Double.NaN;
    private double estimatedPos;
    private double arrivedFor = 0;

    public Hood(HardwareMap hw) {
        servo = hw.get(Servo.class, "hood");
        estimatedPos = angleToPosition(RobotConfig.HOOD_MAX_ANGLE_DEG);
    }

    public static double angleToPosition(double angleDeg) {
        double slope = (RobotConfig.HOOD_CAL_POS_B - RobotConfig.HOOD_CAL_POS_A)
                / (RobotConfig.HOOD_CAL_ANGLE_B_DEG - RobotConfig.HOOD_CAL_ANGLE_A_DEG);
        return RobotConfig.HOOD_CAL_POS_A + (angleDeg - RobotConfig.HOOD_CAL_ANGLE_A_DEG) * slope;
    }

    public static double positionToAngle(double pos) {
        double slope = (RobotConfig.HOOD_CAL_ANGLE_B_DEG - RobotConfig.HOOD_CAL_ANGLE_A_DEG)
                / (RobotConfig.HOOD_CAL_POS_B - RobotConfig.HOOD_CAL_POS_A);
        return RobotConfig.HOOD_CAL_ANGLE_A_DEG + (pos - RobotConfig.HOOD_CAL_POS_A) * slope;
    }

    public void setAngleDeg(double angleDeg) {
        angleDeg = Math.max(RobotConfig.HOOD_MIN_ANGLE_DEG, Math.min(RobotConfig.HOOD_MAX_ANGLE_DEG, angleDeg));
        setPosition(angleToPosition(angleDeg));
    }

    /** Raw servo position, used for calibration. */
    public void setPosition(double pos) {
        pos = Math.max(0, Math.min(1, pos));
        // Skip tiny changes: they don't move the servo but would reset the settle timer.
        if (Double.isNaN(commandedPos) || Math.abs(pos - commandedPos) > 0.0005) {
            commandedPos = pos;
            servo.setPosition(pos);
        }
    }

    /** Call every loop. */
    public void update(double dt) {
        if (Double.isNaN(commandedPos)) return;
        double step = RobotConfig.HOOD_SERVO_SPEED * dt;
        double err = commandedPos - estimatedPos;
        if (Math.abs(err) <= step) {
            estimatedPos = commandedPos;
            arrivedFor += dt;
        } else {
            estimatedPos += Math.copySign(step, err);
            arrivedFor = 0;
        }
    }

    public boolean settled() {
        return !Double.isNaN(commandedPos) && arrivedFor >= RobotConfig.HOOD_SETTLE_SEC;
    }

    public double position() { return commandedPos; }

    /** Best guess of the current actual angle. */
    public double angleDeg() { return positionToAngle(estimatedPos); }

    public double targetAngleDeg() { return Double.isNaN(commandedPos) ? Double.NaN : positionToAngle(commandedPos); }
}
