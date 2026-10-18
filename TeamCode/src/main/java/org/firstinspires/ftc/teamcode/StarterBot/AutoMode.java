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
 * Simple autonomous for the goBILDA BIOBUZZ StarterBot (mecanum).
 *
 *   1. (optional) wait a start delay so we don't run into our alliance partner
 *   2. Turn to face the HIVE AprilTag, set flywheel speed from distance (projectile motion), and shoot the preload
 *   3. Drive off the starting line to score AUTO LEAVE
 *
 * Set up the robot roughly facing the HIVE with its tag in view (telemetry shows "HIVE target" during INIT).
 * If no tag is seen it shoots straight ahead at the fixed TeleOp speed, so the old setup still works.
 *
 * Options can be changed on the gamepad during INIT (shown on telemetry):
 *   Dpad up/down     start delay +/- 1 second
 *   Dpad left/right  leave direction (forward / backward / strafe left / strafe right)
 *   Y                toggle shooting on/off (drive-only auto if the shooter is broken)
 *   X                alliance RED / BLUE (which HIVE tags to aim at)
 */
@Autonomous(name = "StarterBot Auto", group = "StarterBot")
public class AutoMode extends LinearOpMode {

    // Tune these on the practice field.
    private static final double SPINUP_TIMEOUT_SEC = 3.0; // give up waiting for speed/aim after this
    private static final double SHOOT_TIME_SEC = 4.0;     // how long to feed (enough to empty the hopper)
    private static final double LEAVE_DISTANCE_IN = 24.0;
    private static final double LEAVE_POWER = 0.4;
    private static final double DRIVE_TIMEOUT_SEC = 4.0;

    private static final String[] LEAVE_NAMES = {"FORWARD", "BACKWARD", "STRAFE LEFT", "STRAFE RIGHT"};

    private final StarterBotHardware robot = new StarterBotHardware();
    private final AimVision vision = new AimVision();
    private final ElapsedTime timer = new ElapsedTime();

    private int startDelaySec = 0;
    private int leaveDirection = 1; // backward: away from the HIVE we just shot into
    private boolean shoot = true;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        vision.init(hardwareMap);

        // Let the drive team pick options while waiting for START.
        while (opModeInInit()) {
            vision.update();
            if (gamepad1.dpadUpWasPressed()) startDelaySec = Math.min(startDelaySec + 1, 15);
            if (gamepad1.dpadDownWasPressed()) startDelaySec = Math.max(startDelaySec - 1, 0);
            if (gamepad1.dpadRightWasPressed()) leaveDirection = (leaveDirection + 1) % LEAVE_NAMES.length;
            if (gamepad1.dpadLeftWasPressed()) leaveDirection = (leaveDirection + LEAVE_NAMES.length - 1) % LEAVE_NAMES.length;
            if (gamepad1.yWasPressed()) shoot = !shoot;
            if (gamepad1.xWasPressed()) vision.setRedAlliance(!vision.isRedAlliance());

            telemetry.addLine("Ready. Change options with the gamepad:");
            telemetry.addData("Start delay (dpad up/down)", "%d s", startDelaySec);
            telemetry.addData("Leave direction (dpad L/R)", LEAVE_NAMES[leaveDirection]);
            telemetry.addData("Shoot preload (Y)", shoot ? "YES" : "NO");
            telemetry.addData("Alliance (X)", vision.isRedAlliance() ? "RED" : "BLUE");
            telemetry.addData("HIVE target", vision.hasTarget()
                    ? String.format("%s  %.1f in  %.1f deg", vision.targetName(), vision.distanceIn(), vision.headingErrorDeg())
                    : "none (will shoot at fixed speed)");
            telemetry.addData("Tags seen", vision.seenIds());
            telemetry.update();
        }
        if (isStopRequested()) {
            vision.close();
            return;
        }

        waitSeconds(startDelaySec, "Start delay");

        if (shoot) shootPreload();

        leave();

        robot.stopAll();
        vision.close();
        telemetry.addData("Auto", "Done");
        telemetry.update();
    }

    private void shootPreload() {
        // Spin up and turn toward the HIVE at the same time.
        timer.reset();
        while (opModeIsActive() && !aimAndCheckReady() && timer.seconds() < SPINUP_TIMEOUT_SEC) {
            robot.setFeeder(0);
            telemetry.addData("Auto", "Aiming / spinning up: %.0f", robot.launcher.getVelocity());
            telemetry.update();
        }

        timer.reset();
        while (opModeIsActive() && timer.seconds() < SHOOT_TIME_SEC) {
            // Keep aiming and only feed while on target and at speed so every shot matches;
            // pause feeding if the flywheel dips after a shot.
            if (aimAndCheckReady()) {
                robot.setFeeder(1);
                robot.setIntake(StarterBotHardware.INTAKE_SHOOT_BOOST);
            } else {
                robot.setFeeder(0);
                robot.setIntake(0);
            }
            telemetry.addData("Auto", "Shooting: %.0f", robot.launcher.getVelocity());
            telemetry.update();
        }

        robot.drive(0, 0, 0);
        robot.setFeeder(0);
        robot.setIntake(0);
        robot.setLauncherVelocity(0);
    }

    /**
     * One step of aiming: turn toward the HIVE tag and set the flywheel speed for its distance.
     * Falls back to the fixed speed and no turning when the tag isn't visible or the shot is out of range.
     * Returns true when it's OK to feed.
     */
    private boolean aimAndCheckReady() {
        vision.update();
        Ballistics.Shot shot = vision.hasTarget()
                ? Ballistics.solve(vision.distanceIn(), Ballistics.DEFAULT_LAUNCH_EFFICIENCY) : null;

        if (shot != null && shot.valid) {
            robot.drive(0, 0, StarterBotHardware.aimTurnPower(vision.headingErrorDeg()));
            robot.setLauncherVelocity(shot.ticksPerSec);
            telemetry.addData("Aim", "%.1f in  %.1f deg  target %.0f",
                    vision.distanceIn(), vision.headingErrorDeg(), shot.ticksPerSec);
            return Math.abs(vision.headingErrorDeg()) <= StarterBotHardware.AIM_TOLERANCE_DEG
                    && robot.launcherAtSpeed(shot.ticksPerSec);
        }

        robot.drive(0, 0, 0);
        robot.setLauncherVelocity(StarterBotHardware.LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Aim", vision.hasTarget() ? "OUT OF RANGE - fixed speed" : "no tag - fixed speed");
        return robot.launcherReady();
    }

    private void leave() {
        double forward = 0, strafe = 0;
        switch (leaveDirection) {
            case 0: forward = LEAVE_POWER; break;
            case 1: forward = -LEAVE_POWER; break;
            case 2: strafe = -LEAVE_POWER; break;
            case 3: strafe = LEAVE_POWER; break;
        }

        robot.resetDriveEncoders();
        timer.reset();
        while (opModeIsActive()
                && robot.averageDriveInches() < LEAVE_DISTANCE_IN
                && timer.seconds() < DRIVE_TIMEOUT_SEC) {
            robot.drive(forward, strafe, 0);
            telemetry.addData("Auto", "Leaving: %.1f / %.1f in", robot.averageDriveInches(), LEAVE_DISTANCE_IN);
            telemetry.update();
        }
        robot.drive(0, 0, 0);
    }

    private void waitSeconds(double seconds, String label) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            telemetry.addData("Auto", "%s: %.1f", label, seconds - timer.seconds());
            telemetry.update();
        }
    }
}
