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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Driver-controlled OpMode for the goBILDA BIOBUZZ StarterBot (mecanum).
 *
 * CONTROLS (gamepad1 = driver, gamepad2 = optional operator; mechanism buttons work on either)
 *   Left stick          Drive forward/back + strafe
 *   Right stick X       Turn
 *   Left bumper (hold)  Slow mode for lining up with the HIVE / FLOWER
 *
 *   Right trigger       Intake POLLEN (proportional)
 *   Left trigger        Reverse intake / spit out (proportional)
 *
 *   Y                   Toggle launcher flywheel spin-up on/off (pre-spin while driving to the HIVE)
 *   Right bumper (hold) SHOOT: auto-aims at the HIVE, sets flywheel speed from distance, feeds once
 *                       aimed and at speed. With no HIVE tag in view it shoots at the old fixed speed.
 *   A (hold, gamepad1)  Auto-aim only: turns to face the HIVE while you keep driving with the left stick
 *   B                   Stop the launcher
 *   X (hold)            Run windmill feeder backwards to clear a jam
 *
 *   X during INIT (gamepad1) Choose alliance (which HIVE tags to aim at)
 *   Dpad up/down (gamepad1)  Calibrate launch efficiency +/- 0.01 (shots short -> down, long -> up).
 *                            Copy the final value into Ballistics.DEFAULT_LAUNCH_EFFICIENCY.
 *
 * AprilTag aiming: the webcam finds our alliance's raised CELL by the tags under it, AimVision turns that into distance + heading error,
 * and Ballistics uses projectile motion to pick the flywheel speed for that distance.
 */
@TeleOp(name = "StarterBot TeleOp", group = "StarterBot")
public class StarterBotTeleOp extends OpMode {

    private static final double SLOW_MODE_SCALE = 0.35;
    private static final double STICK_DEADZONE = 0.05;

    private final StarterBotHardware robot = new StarterBotHardware();
    private final AimVision vision = new AimVision();
    private boolean launcherOn = false;
    private double efficiency = Ballistics.DEFAULT_LAUNCH_EFFICIENCY;

    @Override
    public void init() {
        robot.init(hardwareMap);
        vision.init(hardwareMap);
        telemetry.addData("Status", "Initialized - check config names if anything is missing");
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) vision.setRedAlliance(!vision.isRedAlliance());
        vision.update();
        telemetry.addData("Alliance (X)", vision.isRedAlliance() ? "RED" : "BLUE");
        telemetry.addData("HIVE target", vision.targetName());
        telemetry.addData("Tags seen", vision.seenIds());
    }

    @Override
    public void loop() {
        // ---------------- Vision + shot solution ----------------
        vision.update();
        if (gamepad1.dpadUpWasPressed()) efficiency = Math.min(efficiency + 0.01, 1.0);
        if (gamepad1.dpadDownWasPressed()) efficiency = Math.max(efficiency - 0.01, 0.05);

        Ballistics.Shot shot = vision.hasTarget() ? Ballistics.solve(vision.distanceIn(), efficiency) : null;
        boolean aimable = shot != null && shot.valid;
        boolean aimed = vision.hasTarget() && Math.abs(vision.headingErrorDeg()) <= StarterBotHardware.AIM_TOLERANCE_DEG;
        boolean shooting = gamepad1.right_bumper || gamepad2.right_bumper;

        // ---------------- Drive ----------------
        double forward = deadzone(-gamepad1.left_stick_y); // stick forward is negative
        double strafe = deadzone(gamepad1.left_stick_x);
        double turn = deadzone(gamepad1.right_stick_x);

        boolean slow = gamepad1.left_bumper;
        double scale = slow ? SLOW_MODE_SCALE : 1.0;
        boolean autoAim = (gamepad1.a || shooting) && vision.hasTarget();
        turn = autoAim ? StarterBotHardware.aimTurnPower(vision.headingErrorDeg()) : turn * scale;
        robot.drive(forward * scale, strafe * scale, turn);

        // ---------------- Launcher ----------------
        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
            launcherOn = !launcherOn;
        }
        if (gamepad1.b || gamepad2.b) {
            launcherOn = false;
        }

        if (shooting) {
            launcherOn = true; // stays spinning after you let go so the next shot is quick
        }
        // Distance-based speed when we can see the HIVE, otherwise the old fixed speed.
        double targetVelocity = aimable ? shot.ticksPerSec : StarterBotHardware.LAUNCHER_TARGET_VELOCITY;
        robot.setLauncherVelocity(launcherOn ? targetVelocity : 0);
        boolean readyToFire = aimable
                ? aimed && robot.launcherAtSpeed(targetVelocity)
                : robot.launcherReady();

        // ---------------- Intake + Feeder ----------------
        double intakePower = Math.max(gamepad1.right_trigger, gamepad2.right_trigger)
                - Math.max(gamepad1.left_trigger, gamepad2.left_trigger);

        double feederPower = 0;
        if (gamepad1.x || gamepad2.x) {
            feederPower = -1; // unjam
        } else if (shooting && readyToFire) {
            feederPower = 1;
            intakePower += StarterBotHardware.INTAKE_SHOOT_BOOST;
        }
        robot.setFeeder(feederPower);
        robot.setIntake(intakePower);

        // ---------------- Telemetry ----------------
        telemetry.addData("Drive", slow ? "SLOW" : "normal");
        telemetry.addData("Launcher", "%s  vel %.0f / %.0f  %s",
                launcherOn ? "ON" : "off",
                robot.launcher.getVelocity(),
                targetVelocity,
                readyToFire ? "READY" : "");
        telemetry.addData("Intake", "%.2f", Math.max(-1, Math.min(1, intakePower)));
        telemetry.addData("Feeder", "%.2f", feederPower);

        telemetry.addData("HIVE target", vision.targetName());
        telemetry.addData("Tags seen", vision.seenIds());
        if (vision.hasTarget()) {
            telemetry.addData("Distance", "%.1f in  (tag height %.1f in)", vision.distanceIn(), vision.tagHeightIn());
            telemetry.addData("Heading error", "%.1f deg %s", vision.headingErrorDeg(), aimed ? "AIMED" : "");
            telemetry.addData("Shot", aimable
                    ? String.format("exit %.0f in/s  flight %.2f s  entry %.0f deg", shot.exitSpeed, shot.timeOfFlight, shot.entryAngleDeg)
                    : "OUT OF RANGE - using fixed speed");
            // If the old fixed-speed shot scores from here, this is the efficiency to use.
            telemetry.addData("Efficiency if fixed speed scores here", "%.3f",
                    Ballistics.efficiencyFromGoodShot(vision.distanceIn(), StarterBotHardware.LAUNCHER_TARGET_VELOCITY));
        }
        telemetry.addData("Launch efficiency (dpad up/down)", "%.2f", efficiency);
    }

    @Override
    public void stop() {
        robot.stopAll();
        vision.close();
    }

    private static double deadzone(double v) {
        return Math.abs(v) < STICK_DEADZONE ? 0 : v;
    }
}
