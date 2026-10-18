package org.firstinspires.ftc.teamcode.StarterBot;

/*
 * Projectile-motion shot solver for the StarterBot launcher.
 *
 * The launcher has a fixed hood angle, so the only thing we can change is how fast POLLEN leaves the flywheel.
 * Given the horizontal distance d to the HIVE opening and its height above the launcher exit (dh), the
 * no-air-resistance projectile equations are:
 *
 *   x(t) = v cos(theta) t
 *   y(t) = v sin(theta) t - g t^2 / 2
 *
 * Solve x = d for t, plug into y = dh, and solve for v:
 *
 *   v = (d / cos(theta)) * sqrt( g / (2 (d tan(theta) - dh)) )
 *
 * If d tan(theta) <= dh the target is above the straight line the launcher points along, so no speed can reach it.
 *
 * The flywheel doesn't transfer all of its surface speed to the POLLEN (it slips and squishes), so
 *   exitSpeed = LAUNCH_EFFICIENCY * wheelSurfaceSpeed
 * LAUNCH_EFFICIENCY is the one number you calibrate on the field (see StarterBotTeleOp dpad controls).
 *
 * All distances are in inches, speeds in inches/second.
 */
public final class Ballistics {

    public static final double G = 386.088; // gravity, in/s^2

    // From the Competition Manual (TU04, Figure 9-10): the raised CELL's opening runs from 53.5 in to 65.6 in
    // above the tiles. Aim for the middle.
    public static final double HIVE_TARGET_HEIGHT_IN = (53.5 + 65.6) / 2;

    // ----- MEASURE THESE ON YOUR ROBOT -----
    // The opening is high, so the hood angle sets your minimum shooting distance (ball must be coming down):
    //   45 deg -> ~10 ft, 50 deg -> ~8 ft, 55 deg -> ~6.5 ft, 60 deg -> ~5.5 ft, 65 deg -> ~4.5 ft
    public static final double LAUNCH_ANGLE_DEG = 50.0;        // hood exit angle above horizontal
    public static final double LAUNCHER_EXIT_HEIGHT_IN = 12.0; // floor to where POLLEN leaves the flywheel
    public static final double FLYWHEEL_DIAMETER_IN = 4.0;
    public static final double LAUNCHER_TICKS_PER_REV = 28.0;  // goBILDA 6000 RPM motor (1:1)
    public static final double LAUNCHER_MAX_TICKS_PER_SEC = 2700; // a bit under the 6000 RPM free speed (2800)

    // POLLEN has to be falling at least this steeply when it reaches the target to drop into the HIVE.
    // Closer than a certain distance the ball is still rising at the opening, so those shots are rejected.
    // Set to 90 to allow any shot (e.g. if the HIVE opening faces the robot instead of facing up).
    public static final double MAX_ENTRY_ANGLE_DEG = -10.0;

    // Starting guess for exitSpeed / wheelSurfaceSpeed. Tune on the field, then paste the tuned value here.
    public static final double DEFAULT_LAUNCH_EFFICIENCY = 0.50;

    private Ballistics() {}

    /** Exit speed (in/s) needed to hit a point d inches away and dh inches higher. NaN if unreachable. */
    public static double requiredLaunchSpeed(double d, double dh, double thetaDeg) {
        double theta = Math.toRadians(thetaDeg);
        double cos = Math.cos(theta);
        double denom = 2 * cos * cos * (d * Math.tan(theta) - dh);
        if (d <= 0 || denom <= 0) return Double.NaN;
        return Math.sqrt(G * d * d / denom);
    }

    /** Seconds from launch until the POLLEN has travelled d inches horizontally. */
    public static double timeOfFlight(double d, double v, double thetaDeg) {
        return d / (v * Math.cos(Math.toRadians(thetaDeg)));
    }

    /** Angle of travel when the POLLEN reaches the target, degrees (negative = coming down, which is what we want). */
    public static double entryAngleDeg(double d, double v, double thetaDeg) {
        double theta = Math.toRadians(thetaDeg);
        double t = timeOfFlight(d, v, thetaDeg);
        double vx = v * Math.cos(theta);
        double vy = v * Math.sin(theta) - G * t;
        return Math.toDegrees(Math.atan2(vy, vx));
    }

    /** Flywheel velocity (encoder ticks/s) that produces the given exit speed. */
    public static double exitSpeedToTicksPerSec(double exitSpeed, double efficiency) {
        double wheelSurfaceSpeed = exitSpeed / efficiency;
        double revsPerSec = wheelSurfaceSpeed / (Math.PI * FLYWHEEL_DIAMETER_IN);
        return revsPerSec * LAUNCHER_TICKS_PER_REV;
    }

    /** Result of solving a shot from a given horizontal distance. */
    public static class Shot {
        public final boolean valid;
        public final double distanceIn;
        public final double exitSpeed;      // in/s
        public final double ticksPerSec;    // flywheel target
        public final double timeOfFlight;   // s
        public final double entryAngleDeg;

        Shot(boolean valid, double distanceIn, double exitSpeed, double ticksPerSec, double timeOfFlight, double entryAngleDeg) {
            this.valid = valid;
            this.distanceIn = distanceIn;
            this.exitSpeed = exitSpeed;
            this.ticksPerSec = ticksPerSec;
            this.timeOfFlight = timeOfFlight;
            this.entryAngleDeg = entryAngleDeg;
        }
    }

    /** Solve for the flywheel speed to hit the HIVE from distanceIn inches away (horizontal, launcher to target). */
    public static Shot solve(double distanceIn, double efficiency) {
        double dh = HIVE_TARGET_HEIGHT_IN - LAUNCHER_EXIT_HEIGHT_IN;
        double v = requiredLaunchSpeed(distanceIn, dh, LAUNCH_ANGLE_DEG);
        if (Double.isNaN(v)) return new Shot(false, distanceIn, 0, 0, 0, 0);

        double ticks = exitSpeedToTicksPerSec(v, efficiency);
        double entry = entryAngleDeg(distanceIn, v, LAUNCH_ANGLE_DEG);
        boolean reachable = ticks <= LAUNCHER_MAX_TICKS_PER_SEC && entry <= MAX_ENTRY_ANGLE_DEG;
        return new Shot(reachable, distanceIn, v, Math.min(ticks, LAUNCHER_MAX_TICKS_PER_SEC),
                timeOfFlight(distanceIn, v, LAUNCH_ANGLE_DEG), entry);
    }

    /**
     * Calibration helper: if a shot from distanceIn actually scores at ticksPerSec, this is the efficiency
     * that makes the model agree. Shoot from a few distances and average the results.
     */
    public static double efficiencyFromGoodShot(double distanceIn, double ticksPerSec) {
        double dh = HIVE_TARGET_HEIGHT_IN - LAUNCHER_EXIT_HEIGHT_IN;
        double v = requiredLaunchSpeed(distanceIn, dh, LAUNCH_ANGLE_DEG);
        double wheelSurfaceSpeed = ticksPerSec / LAUNCHER_TICKS_PER_REV * Math.PI * FLYWHEEL_DIAMETER_IN;
        return v / wheelSurfaceSpeed;
    }
}
