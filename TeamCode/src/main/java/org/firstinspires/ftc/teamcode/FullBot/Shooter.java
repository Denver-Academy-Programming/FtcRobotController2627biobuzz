package org.firstinspires.ftc.teamcode.biobuzz;

/*
 * Turns "where is the CELL" + "where/how fast is the robot" into hood angle, flywheel speed, aim heading,
 * and decides exactly when each ball is fed.
 *
 * Each loop:
 *   1. Launcher position/velocity on the field (robot velocity + spin about the robot center).
 *   2. Shoot-on-the-move: the ball keeps the launcher's velocity, so aim at a virtual target
 *      (real target - launcher velocity x time of flight). Iterated because time of flight depends on distance.
 *   3. ShotSolver gives hood angle + exit speed for that distance.
 *   4. Aim heading: robot heading that points the launcher (which is offset from the center) at the target.
 *   5. READY only when: solution valid, flywheel both sides stable at speed, hood arrived, heading within the
 *      angle the CELL opening subtends at this distance, and not still rotating fast.
 *   6. FIRE feeds one ball at a time: feed while READY, stop the instant the flywheel sees the ball go through,
 *      wait for recovery, repeat. Every ball leaves at the same speed.
 */
public class Shooter {

    /** MANUAL = compute the solution and aim but leave flywheel/hood to the caller (tuning). */
    public enum Mode { OFF, MANUAL, SPIN, AIM, FIRE }

    // Flywheel speed while pre-spinning with no valid solution (saves recovery time when one appears).
    public static double IDLE_TICKS = 1300;   // [PLACEHOLDER] TUNE: about the speed of your most common shot

    private final Flywheel flywheel;
    private final Hood hood;
    private final HiveVision vision;
    private final ShotSolver solver;

    private Mode mode = Mode.OFF;
    private ShotSolver.Solution solution;
    private double aimHeading = Double.NaN;
    private double distance = Double.NaN;
    private double headingTolerance = 0;
    private boolean ready = false;
    private boolean feeding = false;
    private double feedHold = 0;
    private int lastShotCount = 0;
    private int shotsThisVolley = 0;

    // Driver-selected fallback distance when no target is known (NaN = none).
    private double manualDistance = Double.NaN;

    public Shooter(Flywheel flywheel, Hood hood, HiveVision vision, ShotSolver solver) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.vision = vision;
        this.solver = solver;
    }

    public void setMode(Mode m) {
        if (m == Mode.FIRE && mode != Mode.FIRE) shotsThisVolley = 0;
        mode = m;
    }

    public void setManualDistance(double inches) { manualDistance = inches; }
    public double manualDistance() { return manualDistance; }

    public void update(Drivetrain drive, double dt) {
        Pose pose = drive.pose();
        boolean haveTarget = vision.hasTarget();
        solution = null;
        aimHeading = Double.NaN;
        distance = Double.NaN;

        if (haveTarget) {
            computeAim(drive, pose);
        } else if (!Double.isNaN(manualDistance)) {
            distance = manualDistance + RobotConfig.DISTANCE_TRIM_IN;
            solution = solver.solve(distance);
        }
        boolean valid = solution != null && solution.valid;

        // ---- Actuators ----
        if (mode == Mode.MANUAL) {
            // caller drives the flywheel and hood
        } else if (mode == Mode.OFF) {
            flywheel.setTarget(0);
        } else if (valid) {
            flywheel.setTarget(solution.ticksPerSec());
            hood.setAngleDeg(solution.angleDeg);
        } else {
            flywheel.setTarget(IDLE_TICKS);
        }

        // ---- Ready? ----
        boolean aimed = Double.isNaN(aimHeading)  // manual shot: driver aims
                || (Math.abs(drive.headingError(aimHeading)) <= headingTolerance
                && Math.abs(drive.omega()) <= Math.toRadians(RobotConfig.MAX_AIM_TURN_RATE_DEG_PER_SEC));
        ready = (valid || mode == Mode.MANUAL) && mode != Mode.OFF && flywheel.ready() && hood.settled() && aimed;

        // ---- Feed one ball at a time ----
        int shots = flywheel.shotCount();
        if (shots != lastShotCount) {
            shotsThisVolley += shots - lastShotCount;
            lastShotCount = shots;
            feedHold = RobotConfig.MIN_SHOT_SPACING_SEC;
        }
        if (feedHold > 0) feedHold -= dt;
        feeding = mode == Mode.FIRE && ready && feedHold <= 0;
    }

    private void computeAim(Drivetrain drive, Pose pose) {
        double tX = vision.targetX(), tY = vision.targetY();
        double lf = RobotConfig.LAUNCHER_FORWARD_IN, ll = RobotConfig.LAUNCHER_LEFT_IN;
        double launcherYaw = Math.toRadians(RobotConfig.LAUNCHER_YAW_DEG);

        // Launcher field velocity = robot velocity + omega x r
        double[] lp = pose.toField(lf, ll);
        double rx = lp[0] - pose.x, ry = lp[1] - pose.y;
        double lvx = drive.vx() - drive.omega() * ry;
        double lvy = drive.vy() + drive.omega() * rx;
        if (!RobotConfig.SHOOT_ON_MOVE) { lvx = 0; lvy = 0; }

        double heading = pose.heading;
        double vtX = tX, vtY = tY;
        ShotSolver.Solution s = null;
        for (int i = 0; i < 3; i++) {
            Pose atHeading = new Pose(pose.x, pose.y, heading);
            lp = atHeading.toField(lf, ll);
            double d = Math.hypot(vtX - lp[0], vtY - lp[1]) + RobotConfig.DISTANCE_TRIM_IN;
            s = solver.solve(d);
            double tof = s.valid ? s.timeOfFlight : 0;
            vtX = tX - lvx * tof;
            vtY = tY - lvy * tof;
            heading = Math.atan2(vtY - lp[1], vtX - lp[0]) - launcherYaw + Math.toRadians(RobotConfig.AIM_TRIM_DEG);
            distance = d;
        }
        solution = s;
        aimHeading = Pose.wrap(heading);
        // Allowed heading error: the angle half the opening subtends, never tighter than 0.75 deg (IMU/servo noise).
        headingTolerance = Math.max(Math.toRadians(0.75), Math.atan2(RobotConfig.HIVE_AIM_HALF_WIDTH_IN, Math.max(distance, 1)));
    }

    /** Turn power to hold the aim heading, or NaN if there's nothing to aim at (driver keeps control). */
    public double aimTurnPower(Drivetrain drive) {
        return Double.isNaN(aimHeading) ? Double.NaN : drive.headingPower(aimHeading);
    }

    public Mode mode() { return mode; }
    public boolean ready() { return ready; }
    public boolean feeding() { return feeding; }
    public int shotsThisVolley() { return shotsThisVolley; }
    public ShotSolver.Solution solution() { return solution; }
    public double distance() { return distance; }
    public double aimHeading() { return aimHeading; }
    public double headingToleranceDeg() { return Math.toDegrees(headingTolerance); }

    /** One-line reason the shooter isn't ready, for telemetry. */
    public String status(Drivetrain drive) {
        if (mode == Mode.OFF) return "off";
        if (solution == null) return "no target (no tag, no manual distance)";
        if (!solution.valid) return "can't shoot: " + solution.reason;
        if (!flywheel.present()) return "no launcher motors configured";
        if (!flywheel.ready()) return "spinning up";
        if (!hood.settled()) return "hood moving";
        if (!Double.isNaN(aimHeading) && Math.abs(drive.headingError(aimHeading)) > headingTolerance) return "turning to aim";
        if (!ready) return "settling rotation";
        return feeding ? "FIRING" : "READY";
    }
}
