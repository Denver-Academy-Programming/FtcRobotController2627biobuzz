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
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Driver-controlled OpMode for the goBILDA BIOBUZZ StarterBot (2 rear drive motors, no webcam).
 *
 * CONTROLS (gamepad1 = driver, gamepad2 = optional operator; mechanism buttons work on either)
 *   Left stick Y        Drive forward / back
 *   Right stick X       Turn
 *   Left bumper (hold)  Slow mode for lining up with the HIVE / FLOWER
 *
 *   Right trigger       Intake POLLEN (proportional)
 *   Left trigger        Reverse intake / spit out (proportional)
 *
 *   Y                   Launcher on / off (pre-spin while driving to the HIVE)
 *   B                   Launcher off
 *   Right bumper (hold) SHOOT: turns the launcher on if needed, feeds once it has spun up
 *   X (hold)            Windmill feeder backwards to clear a jam
 *
 *   Dpad up / down      Launcher power +/- 0.01   (shots long -> down, short -> up)
 *   Dpad right / left   Launcher power +/- 0.05
 *                       Copy the final value into StarterBotHardware.LAUNCHER_DEFAULT_POWER (auto uses it too).
 */
@TeleOp(name = "StarterBot TeleOp", group = "StarterBot")
public class StarterBotTeleOp extends OpMode {

    private static final double SLOW_MODE_SCALE = 0.35;
    private static final double STICK_DEADZONE = 0.05;

    private final StarterBotHardware robot = new StarterBotHardware();
    private final ElapsedTime launcherTimer = new ElapsedTime(); // time since the launcher was turned on
    private boolean launcherOn = false;
    private double launcherPower = StarterBotHardware.LAUNCHER_DEFAULT_POWER;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        String missing = robot.missing();
        telemetry.addData("Status", missing.isEmpty() ? "Ready" : "Running without some hardware");
        if (!missing.isEmpty()) telemetry.addData("MISSING (check config names)", missing);
        telemetry.addData("Battery", "%.1f V", robot.batteryVoltage());
    }

    @Override
    public void loop() {
        // ---------------- Launcher power tuning ----------------
        double step = StarterBotHardware.LAUNCHER_POWER_STEP;
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) launcherPower += step;
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) launcherPower -= step;
        if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) launcherPower += 5 * step;
        if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) launcherPower -= 5 * step;
        launcherPower = StarterBotHardware.clip(launcherPower, 0, StarterBotHardware.LAUNCHER_MAX_POWER);

        // ---------------- Drive ----------------
        double forward = deadzone(-gamepad1.left_stick_y); // stick forward is negative
        double turn = deadzone(gamepad1.right_stick_x);
        double scale = gamepad1.left_bumper ? SLOW_MODE_SCALE : 1.0;
        robot.drive(forward * scale, turn * scale);

        // ---------------- Launcher ----------------
        boolean shooting = gamepad1.right_bumper || gamepad2.right_bumper;
        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) setLauncher(!launcherOn);
        if (gamepad1.b || gamepad2.b) setLauncher(false);
        if (shooting && !launcherOn) setLauncher(true); // stays on after you let go so the next shot is quick
        robot.setLauncherPower(launcherOn ? launcherPower : 0);

        boolean spunUp = launcherOn && launcherTimer.seconds() >= StarterBotHardware.LAUNCHER_SPINUP_SEC;

        // ---------------- Intake + Feeder ----------------
        double intakePower = Math.max(gamepad1.right_trigger, gamepad2.right_trigger)
                - Math.max(gamepad1.left_trigger, gamepad2.left_trigger);

        double feederPower = 0;
        if (gamepad1.x || gamepad2.x) {
            feederPower = -1; // unjam
        } else if (shooting && spunUp) {
            feederPower = 1;
            intakePower += StarterBotHardware.INTAKE_SHOOT_BOOST;
        }
        robot.setFeeder(feederPower);
        robot.setIntake(intakePower);

        // ---------------- Telemetry ----------------
        String missing = robot.missing();
        if (!missing.isEmpty()) telemetry.addData("MISSING (check config names)", missing);
        telemetry.addData("Drive", gamepad1.left_bumper ? "SLOW" : "normal");
        telemetry.addData("Launcher", "%s  %s", launcherOn ? "ON" : "off",
                shooting ? (spunUp ? "FIRING" : "spinning up...") : (spunUp ? "READY" : ""));
        telemetry.addData("Launcher power (dpad)", "%.2f", launcherPower);
        telemetry.addData("Launcher speed", "%.0f RPM (0 = no encoder, that's OK)", robot.launcherRpm());
        telemetry.addData("Battery", "%.1f V", robot.batteryVoltage());
        telemetry.addData("Intake", "%.2f", StarterBotHardware.clip(intakePower, -1, 1));
        telemetry.addData("Feeder", "%.2f", feederPower);
    }

    @Override
    public void stop() {
        robot.stopAll();
    }

    private void setLauncher(boolean on) {
        if (on && !launcherOn) launcherTimer.reset();
        launcherOn = on;
    }

    private static double deadzone(double v) {
        return Math.abs(v) < STICK_DEADZONE ? 0 : v;
    }
}
