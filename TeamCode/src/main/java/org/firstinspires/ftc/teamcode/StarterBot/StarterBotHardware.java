/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Shared hardware for the goBILDA BIOBUZZ StarterBot (mecanum version).
 * Both StarterBotTeleOp and AutoMode use this so motor names / directions only live in one place.
 *
 * Robot configuration names (set these on the Driver Station):
 *   Motors:  Front_Left, Front_Right, Rear_Left, Rear_Right, intake, launcher
 *   CR Servos: left_intake_servo, right_intake_servo, windmillServo
 *
 * Mechanisms:
 *   intake + left/right intake servos -> pull POLLEN off the floor / out of the FLOWER into the hopper
 *   windmillServo                     -> feeds POLLEN from the hopper into the launcher
 *   launcher                          -> flywheel that shoots POLLEN into the HIVE
 */
public class StarterBotHardware {

    // Launcher speeds in encoder ticks per second (28 ticks/rev on the goBILDA 6000 RPM motor).
    public static final double LAUNCHER_TARGET_VELOCITY = 1250; // ~2678 RPM
    public static final double LAUNCHER_MIN_VELOCITY = 1200;    // only feed once we're at least this fast
    public static final double LAUNCHER_VELOCITY_TOLERANCE = 40; // auto-aimed shots feed within +/- this of target

    // Auto-aim turning (heading error in degrees -> turn power).
    public static final double AIM_TOLERANCE_DEG = 2.0; // close enough to shoot
    public static final double AIM_TURN_GAIN = 0.015;   // 0.3 power at 20 degrees off
    public static final double AIM_MIN_TURN = 0.06;     // enough to overcome friction for small corrections
    public static final double AIM_MAX_TURN = 0.4;

    // Extra intake power while shooting, helps push stuck POLLEN up to the windmill.
    public static final double INTAKE_SHOOT_BOOST = 0.5;

    // Drive encoder math for goBILDA 5203 312 RPM motors (537.7 ticks/rev) and 104mm mecanum wheels.
    // Change these if your Strafer uses different motors or wheels.
    public static final double DRIVE_TICKS_PER_REV = 537.7;
    public static final double WHEEL_DIAMETER_INCHES = 104.0 / 25.4;
    public static final double DRIVE_TICKS_PER_INCH = DRIVE_TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    public DcMotor frontLeft, frontRight, rearLeft, rearRight;
    public DcMotor intake;
    public DcMotorEx launcher;
    public CRServo leftIntakeServo, rightIntakeServo, windmillServo;

    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "Front_Left");
        frontRight = hardwareMap.get(DcMotor.class, "Front_Right");
        rearLeft = hardwareMap.get(DcMotor.class, "Rear_Left");
        rearRight = hardwareMap.get(DcMotor.class, "Rear_Right");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");

        // Pushing the left stick forward MUST drive the robot forward. Flip these if it doesn't.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, rearLeft, rearRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Closed-loop velocity control on the flywheel, with PIDF tuned by goBILDA for this launcher.
        // If the launcher just runs at full speed, check the encoder cable is plugged in next to the motor.
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        // Both intake servos must pull elements inward, so one is reversed.
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        stopAll();
    }

    /** Robot-centric mecanum drive. forward/strafe/turn are -1..1. */
    public void drive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double rl = forward - strafe + turn;
        double rr = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(rl), Math.abs(rr))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        rearLeft.setPower(rl / max);
        rearRight.setPower(rr / max);
    }

    /** Positive = pull in, negative = spit out. */
    public void setIntake(double power) {
        power = Math.max(-1, Math.min(1, power));
        intake.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    public void setLauncherVelocity(double ticksPerSecond) {
        launcher.setVelocity(ticksPerSecond);
    }

    public boolean launcherReady() {
        return launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY;
    }

    /** True when the flywheel is within tolerance of a specific target (used for distance-based shots). */
    public boolean launcherAtSpeed(double targetTicksPerSec) {
        return Math.abs(launcher.getVelocity() - targetTicksPerSec) <= LAUNCHER_VELOCITY_TOLERANCE;
    }

    /** Turn power for drive() that rotates toward a target headingErrorDeg degrees to the right. */
    public static double aimTurnPower(double headingErrorDeg) {
        if (Math.abs(headingErrorDeg) <= AIM_TOLERANCE_DEG) return 0;
        double turn = headingErrorDeg * AIM_TURN_GAIN;
        turn = Math.copySign(Math.max(Math.abs(turn), AIM_MIN_TURN), turn);
        return Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, turn));
    }

    public void setFeeder(double power) {
        windmillServo.setPower(power);
    }

    public void stopAll() {
        drive(0, 0, 0);
        setIntake(0);
        setFeeder(0);
        launcher.setVelocity(0);
    }

    // ----- Encoder helpers for autonomous -----

    public void resetDriveEncoders() {
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, rearLeft, rearRight}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** Average distance travelled by all four wheels, in inches (absolute value). */
    public double averageDriveInches() {
        double sum = Math.abs(frontLeft.getCurrentPosition()) + Math.abs(frontRight.getCurrentPosition())
                + Math.abs(rearLeft.getCurrentPosition()) + Math.abs(rearRight.getCurrentPosition());
        return (sum / 4.0) / DRIVE_TICKS_PER_INCH;
    }
}
