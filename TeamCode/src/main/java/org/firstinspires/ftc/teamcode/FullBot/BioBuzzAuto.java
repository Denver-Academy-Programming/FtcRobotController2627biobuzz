package org.firstinspires.ftc.teamcode.FullBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * BIOBUZZ autonomous.
 *
 *   1. Optional start delay (alliance partner).
 *   2. Drive to the shooting spot while the flywheel/hood pre-set for the planned distance.
 *   3. Aim with AprilTags (falls back to the planned HIVE position via odometry if no tag is seen), fire the
 *      preload one ball at a time until the hopper is empty.
 *   4. Pickup cycles, only if there's time left: drive to a POLLEN pickup line, intake along it,
 *      return to the shooting spot, fire.
 *   5. Park (AUTO LEAVE).
 *
 * All positions are in inches relative to the robot's START pose (x = forward at start, y = left at start), written
 * for the BLUE alliance. RED mirrors them (y -> -y). MEASURE THESE ON THE FIELD: the numbers below are placeholders
 * that show the shape of the routine. Drive the route in TeleOp and read "Pose" off telemetry to get real values.
 *
 * INIT options (gamepad 1):
 *   X               alliance RED / BLUE          Dpad left/right  start position
 *   Dpad up/down    start delay +/- 1 s          Y                pickup cycles 0..max (0 = preload + park)
 */
@Autonomous(name = "BIOBUZZ Auto", group = "BIOBUZZ")
public class BioBuzzAuto extends LinearOpMode {

    // ---------------- PLANS (BLUE coordinates, relative to start) ----------------
    // [PLACEHOLDER] MEASURE ON THE FIELD: every number in PLANS below is a guess that only shows the routine's shape.
    // Place the robot at the start, drive each step in TeleOp, and copy the "Pose" telemetry into these entries.
    private static final class Plan {
        final String name;
        final double hiveX, hiveY;      // rough CELL position: seeds aiming until a tag is seen
        final Pose shoot;               // where we shoot from
        final Pose[][] pickups;         // each pickup: {approach pose, end pose}, intake runs from approach to end
        final Pose park;

        Plan(String name, double hiveX, double hiveY, Pose shoot, Pose[][] pickups, Pose park) {
            this.name = name;
            this.hiveX = hiveX;
            this.hiveY = hiveY;
            this.shoot = shoot;
            this.pickups = pickups;
            this.park = park;
        }
    }

    private static final Plan[] PLANS = {
            new Plan("NEAR HIVE", 84, 24,                         // [PLACEHOLDER] HIVE position
                    Pose.deg(18, 6, 15),                             // [PLACEHOLDER] shoot pose
                    new Pose[][]{
                            {Pose.deg(30, -12, -90), Pose.deg(30, -36, -90)},
                            {Pose.deg(54, -12, -90), Pose.deg(54, -36, -90)},
                    },
                    Pose.deg(30, -20, 0)),
            new Plan("FAR", 120, 30,                              // [PLACEHOLDER] HIVE position
                    Pose.deg(48, 10, 10),                            // [PLACEHOLDER] shoot pose
                    new Pose[][]{
                            {Pose.deg(24, -12, -90), Pose.deg(24, -36, -90)},
                            {Pose.deg(48, -12, -90), Pose.deg(48, -36, -90)},
                    },
                    Pose.deg(36, -24, 0)),
    };

    // ---------------- Tuning ----------------
    private static final double AUTO_LENGTH_SEC = 30.0;
    private static final double PARK_RESERVE_SEC = 2.5;   // always leave this much to reach the park pose
    private static final double CYCLE_ESTIMATE_SEC = 8.0; // [PLACEHOLDER] TIME one real pickup + shoot cycle
    private static final double DRIVE_POWER = 0.8;
    private static final double INTAKE_DRIVE_POWER = 0.35; // [PLACEHOLDER] TUNE: fastest speed that still picks up reliably
    private static final double INTAKE_DWELL_SEC = 0.35;
    private static final int PRELOAD_COUNT = 3;           // [PLACEHOLDER] check the manual's preload limit
    private static final double VOLLEY_TIMEOUT_SEC = 4.0;
    private static final double EMPTY_TIMEOUT_SEC = 0.9;  // ready this long with no shot -> hopper is empty

    private Robot robot;
    private Plan plan;
    private final ElapsedTime matchTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        int planIdx = 0, delaySec = 0, cycles = PLANS[0].pickups.length;
        boolean pX = false, pY = false, pU = false, pD = false, pL = false, pR = false;

        while (opModeInInit()) {
            if (gamepad1.x && !pX) robot.vision.setRedAlliance(!robot.vision.isRedAlliance());
            if (gamepad1.dpad_right && !pR) planIdx = (planIdx + 1) % PLANS.length;
            if (gamepad1.dpad_left && !pL) planIdx = (planIdx + PLANS.length - 1) % PLANS.length;
            if (gamepad1.dpad_up && !pU) delaySec = Math.min(delaySec + 1, 15);
            if (gamepad1.dpad_down && !pD) delaySec = Math.max(delaySec - 1, 0);
            if (gamepad1.y && !pY) cycles = (cycles + 1) % (PLANS[planIdx].pickups.length + 1);
            pX = gamepad1.x; pY = gamepad1.y; pU = gamepad1.dpad_up; pD = gamepad1.dpad_down; pL = gamepad1.dpad_left; pR = gamepad1.dpad_right;
            cycles = Math.min(cycles, PLANS[planIdx].pickups.length);

            robot.update();
            telemetry.addData("Alliance (X)", robot.vision.isRedAlliance() ? "RED" : "BLUE");
            telemetry.addData("Start (dpad L/R)", PLANS[planIdx].name);
            telemetry.addData("Delay (dpad U/D)", "%d s", delaySec);
            telemetry.addData("Pickup cycles (Y)", cycles);
            telemetry.addData("Solver", robot.solver.tableReady() ? "ready" : "building...");
            telemetry.addData("Tags seen", robot.vision.seenIds());
            telemetry.update();
        }
        if (isStopRequested()) { robot.stop(); return; }

        matchTimer.reset();
        boolean red = robot.vision.isRedAlliance();
        plan = PLANS[planIdx];
        robot.drive.setPose(new Pose(0, 0, 0));
        // Pose history from INIT is in the old frame: forget anything vision saw before START, seed from the plan.
        robot.vision.clearTarget();
        Pose hive = side(new Pose(plan.hiveX, plan.hiveY, 0), red);
        robot.vision.seedTarget(hive.x, hive.y);

        Pose shootPose = side(plan.shoot, red);

        waitFor(delaySec);

        // ---- Preload ----
        robot.shooter.setMode(Shooter.Mode.SPIN);
        driveTo(shootPose, DRIVE_POWER, 4.0);
        volley(shootPose, PRELOAD_COUNT);

        // ---- Cycles ----
        for (int i = 0; i < cycles && opModeIsActive(); i++) {
            if (timeLeft() < CYCLE_ESTIMATE_SEC + PARK_RESERVE_SEC) break;
            Pose approach = side(plan.pickups[i][0], red), end = side(plan.pickups[i][1], red);

            robot.shooter.setMode(Shooter.Mode.OFF); // save battery for the flywheel recovery later
            driveTo(approach, DRIVE_POWER, 3.0);
            robot.setIntake(RobotConfig.INTAKE_POWER);
            driveTo(end, INTAKE_DRIVE_POWER, 3.0);
            waitFor(INTAKE_DWELL_SEC);
            robot.setIntake(0.2); // keep POLLEN seated without pushing it into the flywheel

            robot.shooter.setMode(Shooter.Mode.SPIN);
            driveTo(shootPose, DRIVE_POWER, 4.0);
            robot.setIntake(0);
            volley(shootPose, PRELOAD_COUNT);
        }

        // ---- Park ----
        robot.shooter.setMode(Shooter.Mode.OFF);
        robot.setIntake(0);
        driveTo(side(plan.park, red), DRIVE_POWER, Math.max(0.5, timeLeft() - 0.2));
        robot.stop();
    }

    /** Mirror BLUE-plan poses for RED. */
    private static Pose side(Pose p, boolean red) {
        return red ? p.mirrored() : p;
    }

    private double timeLeft() {
        return AUTO_LENGTH_SEC - matchTimer.seconds();
    }

    private void driveTo(Pose target, double power, double timeoutSec) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < timeoutSec) {
            robot.update();
            if (robot.drive.goTo(target, power, robot.dt())) break;
            status("Driving to " + target);
        }
        robot.drive.stop();
    }

    /** Hold position at the shooting spot, aim, and fire until 'balls' shots or the hopper is empty. */
    private void volley(Pose at, int balls) {
        robot.shooter.setMode(Shooter.Mode.FIRE);
        ElapsedTime t = new ElapsedTime();
        ElapsedTime sinceShot = new ElapsedTime();
        int lastShots = 0;
        while (opModeIsActive() && t.seconds() < VOLLEY_TIMEOUT_SEC && timeLeft() > PARK_RESERVE_SEC) {
            robot.update();
            double aim = robot.shooter.aimHeading();
            robot.drive.goTo(new Pose(at.x, at.y, Double.isNaN(aim) ? at.heading : aim), 0.4, robot.dt());

            int shots = robot.shooter.shotsThisVolley();
            if (shots != lastShots) { lastShots = shots; sinceShot.reset(); }
            if (!robot.shooter.ready()) sinceShot.reset(); // only count "empty" time while we could have fired
            // Empty hopper: ready but nothing leaving (allow longer before the first ball arrives).
            if (shots >= balls || sinceShot.seconds() > (shots > 0 ? EMPTY_TIMEOUT_SEC : 2 * EMPTY_TIMEOUT_SEC)) break;
            status(String.format("Volley: %d / %d", shots, balls));
        }
        robot.shooter.setMode(Shooter.Mode.SPIN);
        robot.drive.stop();
    }

    private void waitFor(double sec) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < sec) {
            robot.update();
            robot.drive.stop();
            status(String.format("Waiting %.1f", sec - t.seconds()));
        }
    }

    private void status(String what) {
        telemetry.addData("Auto", "%s   (%.1f s left)", what, timeLeft());
        robot.addTelemetry(telemetry);
        telemetry.update();
    }
}
