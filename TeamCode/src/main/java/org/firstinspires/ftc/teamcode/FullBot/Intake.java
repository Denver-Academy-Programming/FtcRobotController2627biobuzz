package org.firstinspires.ftc.teamcode.FullBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

/*
 * Intake rollers, named "intake" in the Driver Hub configuration. Works with EITHER:
 *   - a DC motor              (Motors -> any motor type), or
 *   - a continuous rotation servo  (Servos -> "Continuous Rotation Servo").
 * Both are a DcMotorSimple, so the same setPower(-1..1) drives either one.
 * Also the feeder: running it pushes POLLEN from the hopper into the flywheel.
 * Positive pulls in, negative spits out. If it spins the wrong way, flip INTAKE_REVERSED.
 * Not configured -> does nothing.
 */
public class Intake {
    private final DcMotorSimple actuator;
    private double power = 0;

    public Intake(HardwareMap hw) {
        actuator = Hw.opt(hw, DcMotorSimple.class, "intake");
        if (actuator == null) return;
        actuator.setDirection(RobotConfig.INTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (actuator instanceof DcMotor) {
            DcMotor m = (DcMotor) actuator;
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // stop the next ball instantly between shots
        }
        if (actuator instanceof PwmControl) {
            // SDK default pulse range is 600-2400us; most CR servos (goBILDA, REV Smart Servo) accept 500-2500us,
            // so the wider range lets power 1.0 reach the servo's true top speed.
            ((PwmControl) actuator).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
    }

    /** Positive = pull in / feed, negative = spit out. */
    public void set(double p) {
        power = Math.max(-1, Math.min(1, p));
    }

    public void update(double dt) {
        if (actuator != null) actuator.setPower(power);
    }

    public boolean present() { return actuator != null; }

    /** "motor", "servo" or "not configured", for telemetry. */
    public String kind() {
        if (actuator == null) return "not configured";
        return actuator instanceof DcMotor ? "motor" : "servo";
    }

    public double power() { return power; }

    public void stop() {
        power = 0;
        if (actuator != null) actuator.setPower(0);
    }
}
