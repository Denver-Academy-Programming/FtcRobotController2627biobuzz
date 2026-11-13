package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * BIOBUZZ driver control.
 *
 * GAMEPAD 1 (driver)
 *   Left stick          drive (field-centric by default)       Right stick X   turn
 *   Left bumper (hold)  slow mode                              X               toggle field-centric / robot-centric
 *   Back                reset field-centric "forward" to where the robot faces now
 *   Right trigger       intake         Left trigger   spit out
 *   Right bumper (hold) SHOOT: locks heading on the CELL, sets hood + flywheel from distance, fires one ball at
 *                       a time when everything is ready. Keep driving with the left stick: it leads the shot
 *                       for robot motion (shoot on the move).
 *   A (hold)            aim + spin up only (no feeding)
 *   Y                   toggle flywheel pre-spin          B   launcher off
 *   Dpad up/down        launch efficiency trim (shots short -> up, long -> down)
 *   Dpad left/right     fallback distance when no tag is known (off / 48 / 72 / 96 / 120 / 144 in)
 * GAMEPAD 2 (operator, optional): right bumper shoot, A aim, Y pre-spin, B off, triggers intake.
 * INIT: X toggles alliance (defaults to what AUTO used).
 *
 * Rumble = shooter just became READY.
 */
@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")
public class BioBuzzTeleOp extends OpMode {

    private static final double SLOW_SCALE = 0.35;
    private static final double DEADZONE = 0.05;
    private static final double[] PRESETS = {Double.NaN, 48, 72, 96, 120, 144};

    private Robot robot;
    private boolean fieldCentric = true;
    private double driverForwardHeading = 0;
    private boolean preSpin = false;
    private int preset = 0;
    private boolean wasReady = false;

    // Edge detection
    private boolean prevBack, prevX1, prevY, prevUp, prevDown, prevLeft, prevRight;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        if (Robot.lastRedAlliance != null) robot.vision.setRedAlliance(Robot.lastRedAlliance);
        if (Robot.lastPose != null) robot.drive.setPose(Robot.lastPose);
        driverForwardHeading = robot.drive.pose().heading;
    }

    @Override
    public void init_loop() {
        boolean x = gamepad1.x;
        if (x && !prevX1) robot.vision.setRedAlliance(!robot.vision.isRedAlliance());
        prevX1 = x;
        robot.update();
        telemetry.addData("Alliance (X)", robot.vision.isRedAlliance() ? "RED" : "BLUE");
        telemetry.addData("Pose from auto", Robot.lastPose == null ? "none" : Robot.lastPose.toString());
        robot.addTelemetry(telemetry);
    }

    @Override
    public void loop() {
        // ---------------- Buttons ----------------
        if (pressed(gamepad1.back, prevBack)) driverForwardHeading = robot.drive.pose().heading;
        if (pressed(gamepad1.x, prevX1)) fieldCentric = !fieldCentric;
        if (pressed(gamepad1.y || gamepad2.y, prevY)) preSpin = !preSpin;
        if (gamepad1.b || gamepad2.b) preSpin = false;
        if (pressed(gamepad1.dpad_up, prevUp)) RobotConfig.LAUNCH_EFFICIENCY = Math.min(1.0, RobotConfig.LAUNCH_EFFICIENCY + 0.005);
        if (pressed(gamepad1.dpad_down, prevDown)) RobotConfig.LAUNCH_EFFICIENCY = Math.max(0.1, RobotConfig.LAUNCH_EFFICIENCY - 0.005);
        if (pressed(gamepad1.dpad_right, prevRight)) preset = (preset + 1) % PRESETS.length;
        if (pressed(gamepad1.dpad_left, prevLeft)) preset = (preset + PRESETS.length - 1) % PRESETS.length;
        prevBack = gamepad1.back; prevX1 = gamepad1.x; prevY = gamepad1.y || gamepad2.y;
        prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down; prevLeft = gamepad1.dpad_left; prevRight = gamepad1.dpad_right;

        boolean fire = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean aim = fire || gamepad1.a || gamepad2.a;
        if (fire) preSpin = true; // stays spinning after release so the next volley is instant

        Shooter.Mode mode = fire ? Shooter.Mode.FIRE : aim ? Shooter.Mode.AIM : preSpin ? Shooter.Mode.SPIN : Shooter.Mode.OFF;
        robot.shooter.setMode(mode);
        robot.shooter.setManualDistance(PRESETS[preset]);

        // Triggers are on/off: any real press runs the intake at full power (a half-pulled trigger used to mean half speed).
        double inTrig = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
        double outTrig = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
        robot.setIntake(inTrig > 0.1 ? RobotConfig.INTAKE_POWER : outTrig > 0.1 ? -RobotConfig.INTAKE_POWER : 0);

        robot.update();

        // ---------------- Drive ----------------
        double scale = gamepad1.left_bumper ? SLOW_SCALE : 1.0;
        double fwd = dz(-gamepad1.left_stick_y) * scale;
        double left = dz(-gamepad1.left_stick_x) * scale;
        if (RobotConfig.TELEOP_DRIVE_INVERTED) { fwd = -fwd; left = -left; }
        double turn = dz(-gamepad1.right_stick_x) * scale;   // stick right = turn clockwise

        double aimTurn = aim ? robot.shooter.aimTurnPower(robot.drive) : Double.NaN;
        if (!Double.isNaN(aimTurn)) turn = aimTurn;

        if (fieldCentric) {
            // Rotate driver-frame stick into the field frame.
            double c = Math.cos(driverForwardHeading), s = Math.sin(driverForwardHeading);
            robot.drive.driveField(fwd * c - left * s, fwd * s + left * c, turn);
        } else {
            robot.drive.drive(fwd, left, turn);
        }

        boolean ready = robot.shooter.ready();
        if (ready && !wasReady && aim) gamepad1.rumble(120);
        wasReady = ready;

        // ---------------- Telemetry ----------------
        telemetry.addData("Drive", "%s%s", fieldCentric ? "field-centric" : "robot-centric", gamepad1.left_bumper ? "  SLOW" : "");
        telemetry.addData("Mode", "%s   shots this volley %d", mode, robot.shooter.shotsThisVolley());
        telemetry.addData("Fallback distance (dpad L/R)", Double.isNaN(PRESETS[preset]) ? "off" : String.format("%.0f in", PRESETS[preset]));
        telemetry.addData("Efficiency (dpad U/D)", "%.3f", RobotConfig.LAUNCH_EFFICIENCY);
        telemetry.addData("Intake", robot.intake.present() ? String.format("%s  %.2f", robot.intake.kind(), robot.intake.power()) : "not configured");
        robot.addTelemetry(telemetry);
    }

    @Override
    public void stop() {
        robot.stop();
    }

    private static boolean pressed(boolean now, boolean before) {
        return now && !before;
    }

    private static double dz(double v) {
        return Math.abs(v) < DEADZONE ? 0 : v;
    }
}
