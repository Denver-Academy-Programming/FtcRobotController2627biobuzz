package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

/*
 * Shooter calibration. Do these in order, copying each result into RobotConfig:
 *
 *  1. HOOD (Back toggles RAW SERVO mode): dpad left/right moves the raw servo position. Put it at two positions,
 *     measure the exit angle with a phone inclinometer, enter HOOD_CAL_POS_A/B + HOOD_CAL_ANGLE_A/B. Also find the
 *     mechanical limits -> HOOD_MIN/MAX_ANGLE_DEG.
 *  2. FLYWHEEL: Y toggles spin, dpad up/down changes target +/- 50. At several speeds copy the "measured kV" into
 *     FLY_KV. Then feed balls (right bumper) and raise FLY_KP until "recovery" is short without both sides
 *     oscillating. Check "L-R mismatch" stays small.
 *  3. EFFICIENCY + DRAG: put the robot where the tags are visible. A = AUTO (solver picks hood + speed from the
 *     measured distance, robot turns to aim). In MANUAL mode dpad left/right moves the hood +/- 0.5 deg and up/down
 *     the speed: adjust until shots drop dead center, then press X to record the shot.
 *     Record shots at 3+ distances spread out (e.g. 70, 110, 150 in). The fitted efficiency and Cd are shown;
 *     Left bumper applies them live so A mode uses them. B clears the records.
 *     Then copy LAUNCH_EFFICIENCY and BALL_DRAG_CD into RobotConfig.
 */
@TeleOp(name = "BIOBUZZ Shooter Tuner", group = "BIOBUZZ")
public class ShooterTuner extends OpMode {

    private Robot robot;
    private boolean spin = false, auto = false, rawServo = false;
    private double manualTicks = 1500;
    private double manualAngle = 50;
    private double rawPos = 0.5;
    private final List<ShotSolver.GoodShot> shots = new ArrayList<>();
    private double[] fit = null;

    private boolean pA, pB, pX, pY, pBack, pUp, pDown, pLeft, pRight, pLB;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        if (Robot.lastRedAlliance != null) robot.vision.setRedAlliance(Robot.lastRedAlliance);
    }

    @Override
    public void init_loop() {
        if (gamepad1.x && !pX) robot.vision.setRedAlliance(!robot.vision.isRedAlliance());
        pX = gamepad1.x;
        robot.update();
        telemetry.addData("Alliance (X)", robot.vision.isRedAlliance() ? "RED" : "BLUE");
        telemetry.addData("Tags seen", robot.vision.seenIds());
    }

    @Override
    public void loop() {
        if (edge(gamepad1.y, pY)) spin = !spin;
        if (edge(gamepad1.a, pA)) auto = !auto;
        if (edge(gamepad1.back, pBack)) rawServo = !rawServo;
        if (edge(gamepad1.b, pB)) { shots.clear(); fit = null; }

        if (rawServo) {
            if (edge(gamepad1.dpad_right, pRight)) rawPos = Math.min(1, rawPos + 0.005);
            if (edge(gamepad1.dpad_left, pLeft)) rawPos = Math.max(0, rawPos - 0.005);
        } else if (!auto) {
            if (edge(gamepad1.dpad_right, pRight)) manualAngle = Math.min(RobotConfig.HOOD_MAX_ANGLE_DEG, manualAngle + 0.5);
            if (edge(gamepad1.dpad_left, pLeft)) manualAngle = Math.max(RobotConfig.HOOD_MIN_ANGLE_DEG, manualAngle - 0.5);
        }
        if (!auto) {
            if (edge(gamepad1.dpad_up, pUp)) manualTicks = Math.min(RobotConfig.LAUNCHER_MAX_TICKS_PER_SEC, manualTicks + 50);
            if (edge(gamepad1.dpad_down, pDown)) manualTicks = Math.max(0, manualTicks - 50);
        }

        // Record a good shot: distance from vision, what the launcher is actually doing right now.
        if (edge(gamepad1.x, pX) && robot.vision.hasTarget() && !Double.isNaN(robot.shooter.distance())) {
            double vel = 0.5 * (robot.flywheel.velocityLeft() + robot.flywheel.velocityRight());
            shots.add(new ShotSolver.GoodShot(robot.shooter.distance(), vel, robot.hood.targetAngleDeg()));
            fit = ShotSolver.fitEfficiencyAndDrag(shots, RobotConfig.BALL_DRAG_CD);
        }
        if (edge(gamepad1.left_bumper, pLB) && fit != null && !Double.isNaN(fit[0])) {
            RobotConfig.LAUNCH_EFFICIENCY = fit[0];
            RobotConfig.BALL_DRAG_CD = fit[1];
            robot.solver.rebuild(fit[1]);
        }
        pA = gamepad1.a; pB = gamepad1.b; pX = gamepad1.x; pY = gamepad1.y; pBack = gamepad1.back;
        pUp = gamepad1.dpad_up; pDown = gamepad1.dpad_down; pLeft = gamepad1.dpad_left; pRight = gamepad1.dpad_right;
        pLB = gamepad1.left_bumper;

        // ---- Outputs ----
        robot.shooter.setMode(!spin ? Shooter.Mode.OFF : auto ? Shooter.Mode.AIM : Shooter.Mode.MANUAL);
        if (spin && !auto) {
            robot.flywheel.setTarget(manualTicks);
            if (rawServo) robot.hood.setPosition(rawPos); else robot.hood.setAngleDeg(manualAngle);
        } else if (rawServo) {
            robot.hood.setPosition(rawPos);
        }
        robot.setIntake(gamepad1.right_bumper ? RobotConfig.FEED_POWER : gamepad1.right_trigger - gamepad1.left_trigger);
        robot.update();

        double turn = auto ? robot.shooter.aimTurnPower(robot.drive) : Double.NaN;
        robot.drive.drive(0, 0, Double.isNaN(turn) ? -gamepad1.right_stick_x * 0.3 : turn);

        // ---- Telemetry ----
        double vL = robot.flywheel.velocityLeft(), vR = robot.flywheel.velocityRight(), tgt = robot.flywheel.target();
        telemetry.addData("Mode", "%s%s  (A auto, Y spin, Back raw servo)", auto ? "AUTO" : "MANUAL", rawServo ? " RAW SERVO" : "");
        if (rawServo) telemetry.addData("Raw servo pos (dpad L/R)", "%.3f  -> %.1f° with current cal", rawPos, Hood.positionToAngle(rawPos));
        telemetry.addData("Target ticks (dpad U/D)", "%.0f", tgt);
        telemetry.addData("Velocity", "L %.0f  R %.0f   err L %.0f R %.0f   L-R mismatch %.0f", vL, vR, tgt - vL, tgt - vR, vL - vR);
        telemetry.addData("Power", "L %.3f  R %.3f   battery %.2f V", robot.flywheel.powerLeft(), robot.flywheel.powerRight(), robot.volts());
        if (robot.flywheel.ready() && tgt > 0) {
            double p = 0.5 * (robot.flywheel.powerLeft() + robot.flywheel.powerRight());
            telemetry.addData("Measured kV", "1 / %.0f   (copy into FLY_KV)", tgt / Math.max(1e-6, p * robot.volts() / 12.0 - RobotConfig.FLY_KS));
        }
        telemetry.addData("Recovery after last shot", "%.3f s   shots %d", robot.flywheel.lastRecoverySec(), robot.flywheel.shotCount());
        telemetry.addData("Recorded good shots (X)", shots.size());
        for (ShotSolver.GoodShot s : shots) {
            telemetry.addLine(String.format("   %.1f in   %.0f ticks   %.1f°", s.distanceIn, s.ticksPerSec, s.angleDeg));
        }
        if (fit != null) {
            telemetry.addData("FIT (LB applies)", "efficiency %.3f   Cd %.2f   spread %.1f%%", fit[0], fit[1], fit[2] * 100);
        }
        telemetry.addData("Current", "efficiency %.3f   Cd %.2f", RobotConfig.LAUNCH_EFFICIENCY, RobotConfig.BALL_DRAG_CD);
        robot.addTelemetry(telemetry);
    }

    @Override
    public void stop() {
        robot.stop();
    }

    private static boolean edge(boolean now, boolean before) {
        return now && !before;
    }
}
