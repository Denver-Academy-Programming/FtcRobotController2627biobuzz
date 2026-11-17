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
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.List;

/*
 * Shared hardware for the goBILDA BIOBUZZ StarterBot (2 rear drive motors, no webcam).
 * Both StarterBotTeleOp and AutoMode use this so names, directions and tuning live in one place.
 *
 * Robot configuration names (set these on the Driver Station):
 *   Motors:    Rear_Left, Rear_Right, intake, launcher
 *   CR Servos: left_intake_servo, right_intake_servo, windmillServo
 *
 * Every device is optional: if a name is missing from the configuration the OpMode still runs, that part just
 * does nothing, and telemetry lists it under "MISSING". (A device that's configured but unplugged reads 0.)
 */
public class StarterBotHardware {

    // ---------------- Launcher ----------------
    // The launcher runs open-loop (plain power, no encoder needed), corrected for battery voltage so a fresh
    // battery and a tired one shoot the same distance. Power is the fraction of full power at 12 V.
    // Tune live with dpad up/down in TeleOp, then paste the number that scores here.
    public static final double LAUNCHER_DEFAULT_POWER = 0.10;
    public static final double LAUNCHER_POWER_STEP = 0.01;
    public static final double LAUNCHER_MAX_POWER = 1.0;
    public static final double LAUNCHER_SPINUP_SEC = 1.0;   // wait this long after turning on before feeding
    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final double LAUNCHER_TICKS_PER_REV = 28.0; // goBILDA 6000 RPM motor, for the RPM readout only

    // Extra intake power while feeding, helps push stuck POLLEN up to the windmill.
    public static final double INTAKE_SHOOT_BOOST = 0.5;

    // ---------------- Drive ----------------
    // goBILDA 5203 312 RPM motors (537.7 ticks/rev) and 104 mm wheels. Only used by auto for distance.
    public static final double DRIVE_TICKS_PER_REV = 537.7;
    public static final double WHEEL_DIAMETER_INCHES = 104.0 / 25.4;
    public static final double DRIVE_TICKS_PER_INCH = DRIVE_TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    public DcMotor rearLeft, rearRight;
    public DcMotor intake;
    public DcMotorEx launcher;
    public CRServo leftIntakeServo, rightIntakeServo, windmillServo;

    private HardwareMap hardwareMap;
    private final List<String> missing = new ArrayList<>();
    private double launcherPower = 0;

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        missing.clear();

        rearLeft = opt(DcMotor.class, "Rear_Left");
        rearRight = opt(DcMotor.class, "Rear_Right");
        intake = opt(DcMotor.class, "intake");
        launcher = opt(DcMotorEx.class, "launcher");
        leftIntakeServo = opt(CRServo.class, "left_intake_servo");
        rightIntakeServo = opt(CRServo.class, "right_intake_servo");
        windmillServo = opt(CRServo.class, "windmillServo");

        // Pushing the left stick forward MUST drive the robot forward. Flip these if it doesn't.
        if (rearLeft != null) rearLeft.setDirection(DcMotor.Direction.FORWARD);
        if (rearRight != null) rearRight.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotor m : new DcMotor[]{rearLeft, rearRight}) {
            if (m == null) continue;
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (intake != null) {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (launcher != null) {
            // Open loop: with RUN_USING_ENCODER and an unplugged/broken encoder cable the motor runs flat out,
            // which is a classic "launcher way too powerful" cause. Plain power avoids that entirely.
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast down, don't slam the flywheel
        }

        // Both intake servos must pull elements inward, so one is reversed.
        if (rightIntakeServo != null) rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        if (windmillServo != null) windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        stopAll();
    }

    private <T> T opt(Class<? extends T> type, String name) {
        T device = null;
        try {
            device = hardwareMap.tryGet(type, name);
        } catch (RuntimeException ignored) {
            // name exists but is the wrong device type: treat as missing
        }
        if (device == null) missing.add(name);
        return device;
    }

    /** Config names that weren't found, e.g. "intake, launcher". Empty when everything is there. */
    public String missing() {
        return String.join(", ", missing);
    }

    // ---------------- Drive ----------------

    /** Arcade drive for the two rear wheels. forward / turn are -1..1, positive turn = clockwise. */
    public void drive(double forward, double turn) {
        double left = forward + turn;
        double right = forward - turn;
        double max = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        if (rearLeft != null) rearLeft.setPower(left / max);
        if (rearRight != null) rearRight.setPower(right / max);
    }

    public void resetDriveEncoders() {
        for (DcMotor m : new DcMotor[]{rearLeft, rearRight}) {
            if (m == null) continue;
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** Average distance travelled by the two drive wheels, inches (absolute). 0 if encoders aren't plugged in. */
    public double averageDriveInches() {
        double sum = 0;
        int n = 0;
        for (DcMotor m : new DcMotor[]{rearLeft, rearRight}) {
            if (m == null) continue;
            sum += Math.abs(m.getCurrentPosition());
            n++;
        }
        return n == 0 ? 0 : (sum / n) / DRIVE_TICKS_PER_INCH;
    }

    // ---------------- Intake / feeder ----------------

    /** Positive = pull in, negative = spit out. */
    public void setIntake(double power) {
        power = clip(power, -1, 1);
        if (intake != null) intake.setPower(power);
        if (leftIntakeServo != null) leftIntakeServo.setPower(power);
        if (rightIntakeServo != null) rightIntakeServo.setPower(power);
    }

    /** Positive = feed POLLEN into the launcher, negative = back out a jam. */
    public void setFeeder(double power) {
        if (windmillServo != null) windmillServo.setPower(clip(power, -1, 1));
    }

    // ---------------- Launcher ----------------

    /** Launcher power as a fraction of full power at 12 V (0 = off). Compensated for the actual battery voltage. */
    public void setLauncherPower(double power) {
        launcherPower = clip(power, 0, LAUNCHER_MAX_POWER);
        if (launcher == null) return;
        double volts = batteryVoltage();
        double scale = volts > 6 ? NOMINAL_VOLTAGE / volts : 1.0; // ignore bogus readings
        launcher.setPower(clip(launcherPower * scale, 0, 1));
    }

    public double getLauncherPower() {
        return launcherPower;
    }

    /** Flywheel speed in RPM from the encoder, or 0 if the encoder isn't plugged in. */
    public double launcherRpm() {
        if (launcher == null) return 0;
        return launcher.getVelocity() / LAUNCHER_TICKS_PER_REV * 60.0;
    }

    /** Lowest battery voltage reported by any hub, or 0 if none is reporting. */
    public double batteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hardwareMap.voltageSensor) {
            double v = s.getVoltage();
            if (v > 0) min = Math.min(min, v);
        }
        return Double.isInfinite(min) ? 0 : min;
    }

    public void stopAll() {
        drive(0, 0);
        setIntake(0);
        setFeeder(0);
        setLauncherPower(0);
    }

    static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
