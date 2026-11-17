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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Simple autonomous for the goBILDA BIOBUZZ StarterBot (2 rear drive motors, no webcam).
 *
 *   1. (optional) wait a start delay so we don't run into our alliance partner
 *   2. (optional) shoot the preload straight ahead at LAUNCHER_DEFAULT_POWER -> set the robot up aimed at the HIVE
 *   3. drive off the starting line to score AUTO LEAVE
 *
 * Driving uses the wheel encoders for distance. If the encoders aren't plugged in it notices within half a
 * second and falls back to driving for a fixed time, so it still leaves.
 *
 * Options can be changed on gamepad1 during INIT (shown on telemetry):
 *   Dpad up/down     start delay +/- 1 second
 *   Dpad left/right  leave direction (forward / backward)
 *   Y                shoot preload on/off (drive-only auto if the shooter is broken)
 */
@Autonomous(name = "StarterBot Auto", group = "StarterBot")
public class AutoMode extends LinearOpMode {

    // Tune these on the practice field.
    private static final double SHOOT_TIME_SEC = 4.0;      // how long to feed (enough to empty the hopper)
    private static final double LEAVE_DISTANCE_IN = 24.0;
    private static final double LEAVE_POWER = 0.4;
    private static final double LEAVE_TIMEOUT_SEC = 3.0;   // never drive longer than this
    private static final double ENCODER_CHECK_SEC = 0.5;   // if wheels "haven't moved" by now, encoders are unplugged
    private static final double LEAVE_FALLBACK_SEC = 1.0;  // time-based drive when there are no encoders

    private final StarterBotHardware robot = new StarterBotHardware();
    private final ElapsedTime timer = new ElapsedTime();

    private int startDelaySec = 0;
    private boolean leaveForward = false; // backward = away from the HIVE we just shot into
    private boolean shoot = true;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Let the drive team pick options while waiting for START.
        while (opModeInInit()) {
            if (gamepad1.dpadUpWasPressed()) startDelaySec = Math.min(startDelaySec + 1, 15);
            if (gamepad1.dpadDownWasPressed()) startDelaySec = Math.max(startDelaySec - 1, 0);
            if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) leaveForward = !leaveForward;
            if (gamepad1.yWasPressed()) shoot = !shoot;

            String missing = robot.missing();
            if (!missing.isEmpty()) telemetry.addData("MISSING (check config names)", missing);
            telemetry.addLine("Ready. Change options with gamepad1:");
            telemetry.addData("Start delay (dpad up/down)", "%d s", startDelaySec);
            telemetry.addData("Leave direction (dpad L/R)", leaveForward ? "FORWARD" : "BACKWARD");
            telemetry.addData("Shoot preload (Y)", shoot ? "YES  power %.2f" : "NO",
                    StarterBotHardware.LAUNCHER_DEFAULT_POWER);
            telemetry.addData("Battery", "%.1f V", robot.batteryVoltage());
            telemetry.update();
        }
        if (isStopRequested()) return;

        waitSeconds(startDelaySec, "Start delay");
        if (shoot) shootPreload();
        leave();

        robot.stopAll();
        telemetry.addData("Auto", "Done");
        telemetry.update();
    }

    private void shootPreload() {
        robot.setLauncherPower(StarterBotHardware.LAUNCHER_DEFAULT_POWER);
        waitSeconds(StarterBotHardware.LAUNCHER_SPINUP_SEC, "Spinning up");

        timer.reset();
        while (opModeIsActive() && timer.seconds() < SHOOT_TIME_SEC) {
            robot.setLauncherPower(StarterBotHardware.LAUNCHER_DEFAULT_POWER); // re-apply voltage compensation
            robot.setFeeder(1);
            robot.setIntake(StarterBotHardware.INTAKE_SHOOT_BOOST);
            telemetry.addData("Auto", "Shooting %.1f  (%.0f RPM)", SHOOT_TIME_SEC - timer.seconds(), robot.launcherRpm());
            telemetry.update();
        }

        robot.setFeeder(0);
        robot.setIntake(0);
        robot.setLauncherPower(0);
    }

    private void leave() {
        double power = leaveForward ? LEAVE_POWER : -LEAVE_POWER;
        boolean useEncoders = true;

        robot.resetDriveEncoders();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < LEAVE_TIMEOUT_SEC) {
            double inches = robot.averageDriveInches();
            if (useEncoders && timer.seconds() > ENCODER_CHECK_SEC && inches < 0.5) useEncoders = false;

            boolean done = useEncoders ? inches >= LEAVE_DISTANCE_IN : timer.seconds() >= LEAVE_FALLBACK_SEC;
            if (done) break;

            robot.drive(power, 0);
            telemetry.addData("Auto", useEncoders
                    ? String.format("Leaving: %.1f / %.1f in", inches, LEAVE_DISTANCE_IN)
                    : String.format("Leaving (no encoders, timed): %.1f s", timer.seconds()));
            telemetry.update();
        }
        robot.drive(0, 0);
    }

    private void waitSeconds(double seconds, String label) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            telemetry.addData("Auto", "%s: %.1f", label, seconds - timer.seconds());
            telemetry.update();
        }
    }
}
