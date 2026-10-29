package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * Intake rollers. Also the feeder: running it pushes POLLEN from the hopper into the flywheel.
 * Detects jams by motor current and briefly reverses to clear them.
 */
public class Intake {
    private final DcMotorEx motor;
    private double requested = 0;
    private double highCurrentFor = 0;
    private double unjamLeft = 0;
    private double currentA = 0;

    public Intake(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "intake");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // stop the next ball instantly between shots
    }

    /** Positive = pull in / feed, negative = spit out. */
    public void set(double power) {
        requested = Math.max(-1, Math.min(1, power));
    }

    public void update(double dt) {
        // Current isn't in the bulk read, so only pay for the read while pulling in.
        if (requested > 0.3 && unjamLeft <= 0) {
            currentA = motor.getCurrent(CurrentUnit.AMPS);
            highCurrentFor = currentA > RobotConfig.INTAKE_JAM_CURRENT_A ? highCurrentFor + dt : 0;
            if (highCurrentFor > RobotConfig.INTAKE_JAM_TIME_SEC) {
                unjamLeft = RobotConfig.INTAKE_UNJAM_TIME_SEC;
                highCurrentFor = 0;
            }
        } else {
            highCurrentFor = 0;
        }

        if (unjamLeft > 0) {
            unjamLeft -= dt;
            motor.setPower(-0.6);
        } else {
            motor.setPower(requested);
        }
    }

    public boolean unjamming() { return unjamLeft > 0; }
    public double currentAmps() { return currentA; }

    public void stop() {
        requested = 0;
        unjamLeft = 0;
        motor.setPower(0);
    }
}
