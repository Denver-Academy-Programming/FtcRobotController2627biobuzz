package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.List;

/*
 * Projectile shot solver with an adjustable hood: for any distance it picks BOTH the launch angle and the exit speed.
 *
 * ---- Physics ----
 * Vacuum (closed form), launch angle theta, exit speed v, target d inches away and dh inches higher:
 *     x(t) = v cos(theta) t,     y(t) = v sin(theta) t - g t^2 / 2
 *  => v = (d / cos(theta)) * sqrt( g / (2 (d tan(theta) - dh)) )
 *
 * With air drag (quadratic, which is what a foam ball sees):
 *     a = -g z_hat - k |v| v,     k = rho Cd A / (2 m)
 * There's no closed form, so we integrate the flight with RK4 and solve for v numerically. The vacuum answer is
 * the lower bound (drag only ever needs more speed) and makes the root bracket.
 *
 * ---- Choosing the angle ----
 * Many (angle, speed) pairs hit the same point. We pick the one that's most PRECISE: for each candidate angle we
 * perturb speed by SPEED_NOISE_FRACTION and angle by ANGLE_NOISE_DEG, see how far the ball lands from the
 * center of the (upward-facing) CELL opening, and keep the angle with the smallest expected miss.
 * Steep descents and "flat" spots of the range curve win automatically. Shots that aren't descending at least
 * MAX_ENTRY_ANGLE_DEG are rejected because they'd bounce off the CELL rim.
 *
 * That search is too slow for every loop, so a table over SOLVER_MIN..MAX_DISTANCE is built on a background
 * thread at init and interpolated at runtime. Until it's ready, solve() uses a vacuum fallback.
 *
 * Exit speed -> flywheel ticks/s is done at query time with LAUNCH_EFFICIENCY so efficiency can be tuned live.
 * This class has no FTC SDK dependencies so it can be unit-tested on a laptop.
 */
public final class ShotSolver {

    public static final double G = 386.088; // in/s^2
    private static final double AIR_DENSITY_KG_PER_IN3 = 1.204 / 61023.744;
    private static final double DT = 0.004;
    private static final double MAX_FLIGHT_SEC = 3.0;
    private static final double TABLE_STEP_IN = 2.0;
    private static final double COARSE_ANGLE_STEP_DEG = 2.0;

    /** A solved shot. */
    public static final class Solution {
        public final boolean valid;
        public final String reason;       // why it's invalid, or "ok"
        public final double distanceIn;
        public final double angleDeg;     // hood launch angle
        public final double exitSpeed;    // in/s
        public final double timeOfFlight; // s
        public final double entryAngleDeg;
        public final double expectedMissIn;

        Solution(boolean valid, String reason, double distanceIn, double angleDeg, double exitSpeed,
                 double timeOfFlight, double entryAngleDeg, double expectedMissIn) {
            this.valid = valid;
            this.reason = reason;
            this.distanceIn = distanceIn;
            this.angleDeg = angleDeg;
            this.exitSpeed = exitSpeed;
            this.timeOfFlight = timeOfFlight;
            this.entryAngleDeg = entryAngleDeg;
            this.expectedMissIn = expectedMissIn;
        }

        static Solution invalid(double d, String why) {
            return new Solution(false, why, d, 0, 0, 0, 0, 0);
        }

        /** Flywheel velocity target for this shot (ticks/s of the launcher motors). */
        public double ticksPerSec() {
            return exitSpeedToTicks(exitSpeed, RobotConfig.LAUNCH_EFFICIENCY);
        }
    }

    // ---- Parameters captured when the table is built ----
    private final double dh;
    private double k;
    private final double minAngle, maxAngle, maxEntry;
    private final double vMax;
    private final double speedNoise, angleNoise;

    private volatile Solution[] table;
    private volatile boolean building;

    public ShotSolver() {
        this(RobotConfig.BALL_DRAG_CD);
    }

    public ShotSolver(double dragCd) {
        dh = RobotConfig.HIVE_TARGET_HEIGHT_IN - RobotConfig.LAUNCHER_EXIT_HEIGHT_IN;
        k = dragK(dragCd);
        minAngle = RobotConfig.HOOD_MIN_ANGLE_DEG;
        maxAngle = RobotConfig.HOOD_MAX_ANGLE_DEG;
        maxEntry = RobotConfig.MAX_ENTRY_ANGLE_DEG;
        // Build with a generous ceiling; the real (efficiency-dependent) limit is checked at query time.
        vMax = ticksToExitSpeed(RobotConfig.LAUNCHER_MAX_TICKS_PER_SEC, Math.min(1.0, RobotConfig.LAUNCH_EFFICIENCY * 1.3));
        speedNoise = RobotConfig.SPEED_NOISE_FRACTION;
        angleNoise = RobotConfig.ANGLE_NOISE_DEG;
    }

    /** Start building the lookup table on a background thread. */
    public void buildTableAsync() {
        if (building || table != null) return;
        building = true;
        Thread t = new Thread(() -> {
            table = buildTable();
            building = false;
        }, "ShotSolverTable");
        t.setPriority(Thread.MIN_PRIORITY);
        t.setDaemon(true);
        t.start();
    }

    /** Rebuild with a new drag coefficient (used by the tuner after fitting). Vacuum fallback until done. */
    public synchronized void rebuild(double dragCd) {
        if (building) return;
        k = dragK(dragCd);
        table = null;
        buildTableAsync();
    }

    public boolean tableReady() {
        return table != null;
    }

    Solution[] buildTable() {
        int n = (int) Math.round((RobotConfig.SOLVER_MAX_DISTANCE_IN - RobotConfig.SOLVER_MIN_DISTANCE_IN) / TABLE_STEP_IN) + 1;
        Solution[] out = new Solution[n];
        for (int i = 0; i < n; i++) out[i] = solveExact(RobotConfig.SOLVER_MIN_DISTANCE_IN + i * TABLE_STEP_IN);
        return out;
    }

    /** Fast runtime solve: interpolate the table (or vacuum fallback before it's ready). */
    public Solution solve(double distanceIn) {
        Solution s = lookup(distanceIn);
        if (!s.valid) return s;
        if (s.ticksPerSec() > RobotConfig.LAUNCHER_MAX_TICKS_PER_SEC) return Solution.invalid(distanceIn, "too far for flywheel");
        return s;
    }

    private Solution lookup(double d) {
        Solution[] t = table;
        if (t == null) return solveVacuumFallback(d);
        double f = (d - RobotConfig.SOLVER_MIN_DISTANCE_IN) / TABLE_STEP_IN;
        if (f < 0) return Solution.invalid(d, "too close");
        if (f > t.length - 1) return Solution.invalid(d, "too far");
        int i = Math.min((int) Math.floor(f), t.length - 2);
        double w = f - i;
        Solution a = t[i], b = t[i + 1];
        if (!a.valid || !b.valid) {
            Solution near = w < 0.5 ? a : b;
            if (near.valid && Math.min(w, 1 - w) < 0.25) return near;
            return Solution.invalid(d, a.valid ? b.reason : a.reason);
        }
        return new Solution(true, "ok", d,
                lerp(a.angleDeg, b.angleDeg, w), lerp(a.exitSpeed, b.exitSpeed, w),
                lerp(a.timeOfFlight, b.timeOfFlight, w), lerp(a.entryAngleDeg, b.entryAngleDeg, w),
                lerp(a.expectedMissIn, b.expectedMissIn, w));
    }

    /** Full search at one distance: best-precision angle, then exact speed. ~10-40 ms. */
    public Solution solveExact(double d) {
        if (d <= 1) return Solution.invalid(d, "too close");
        double bestAngle = Double.NaN, bestCost = Double.MAX_VALUE;

        for (double a = minAngle; a <= maxAngle + 1e-9; a += COARSE_ANGLE_STEP_DEG) {
            double c = cost(d, a);
            if (c < bestCost) { bestCost = c; bestAngle = a; }
        }
        if (Double.isNaN(bestAngle)) {
            // Diagnose for telemetry.
            boolean belowLaunchLine = d * Math.tan(Math.toRadians(maxAngle)) <= dh;
            boolean tooFast = !belowLaunchLine && Double.isNaN(speedFor(d, Math.toRadians(maxAngle)));
            String why = tooFast ? "too far for flywheel" : "too close (ball still rising)";
            return Solution.invalid(d, why);
        }

        // Golden-section refine around the coarse best. Cost is MAX_VALUE for invalid angles, which golden section tolerates.
        double lo = Math.max(minAngle, bestAngle - COARSE_ANGLE_STEP_DEG);
        double hi = Math.min(maxAngle, bestAngle + COARSE_ANGLE_STEP_DEG);
        final double phi = (Math.sqrt(5) - 1) / 2;
        double x1 = hi - phi * (hi - lo), x2 = lo + phi * (hi - lo);
        double f1 = cost(d, x1), f2 = cost(d, x2);
        for (int i = 0; i < 12; i++) {
            if (f1 < f2) { hi = x2; x2 = x1; f2 = f1; x1 = hi - phi * (hi - lo); f1 = cost(d, x1); }
            else         { lo = x1; x1 = x2; f1 = f2; x2 = lo + phi * (hi - lo); f2 = cost(d, x2); }
        }
        double angle = f1 < f2 ? x1 : x2;
        if (Math.min(f1, f2) > bestCost) angle = bestAngle;

        double theta = Math.toRadians(angle);
        double v = speedFor(d, theta);
        double[] st = new double[4];
        if (Double.isNaN(v) || !flyTo(d, v, theta, k, st)) return Solution.invalid(d, "solver failed");
        double entry = Math.toDegrees(Math.atan2(st[2], st[1]));
        return new Solution(true, "ok", d, angle, v, st[3], entry, cost(d, angle));
    }

    /** Expected horizontal miss (in) at the CELL for this angle, or MAX_VALUE if the angle can't make the shot. */
    double cost(double d, double angleDeg) {
        double theta = Math.toRadians(angleDeg);
        double v = speedFor(d, theta);
        if (Double.isNaN(v)) return Double.MAX_VALUE;
        double[] st = new double[4];
        if (!flyTo(d, v, theta, k, st)) return Double.MAX_VALUE;
        double entry = Math.atan2(st[2], st[1]);
        if (Math.toDegrees(entry) > maxEntry) return Double.MAX_VALUE;
        double slope = Math.abs(Math.tan(entry)); // converts a height error at the target into a horizontal miss

        double yV = flyTo(d, v * (1 + speedNoise), theta, k, st) ? st[0] : dh + 100;
        double yA = flyTo(d, v, theta + Math.toRadians(angleNoise), k, st) ? st[0] : dh + 100;
        double missV = (yV - dh) / slope, missA = (yA - dh) / slope;
        return Math.hypot(missV, missA);
    }

    /** Exit speed that puts the ball exactly at (d, dh) for launch angle theta, or NaN. */
    double speedFor(double d, double theta) {
        return speedFor(d, dh, theta, k, vMax);
    }

    static double speedFor(double d, double dh, double theta, double k, double vMax) {
        double v0 = vacuumSpeed(d, dh, theta);
        if (Double.isNaN(v0) || v0 > vMax) return Double.NaN;
        if (k <= 0) return v0;

        double[] st = new double[4];
        double lo = v0, hi = vMax;
        double fHi = flyTo(d, hi, theta, k, st) ? st[0] - dh : -1e9;
        if (fHi < 0) return Double.NaN;
        double fLo = flyTo(d, lo, theta, k, st) ? st[0] - dh : Double.NaN;

        // Illinois false-position; bisection while the low end doesn't reach the target at all.
        int side = 0;
        for (int i = 0; i < 60 && hi - lo > 0.01; i++) {
            double m = Double.isNaN(fLo) ? 0.5 * (lo + hi) : hi - fHi * (hi - lo) / (fHi - fLo);
            if (!(m > lo && m < hi)) m = 0.5 * (lo + hi);
            double fm = flyTo(d, m, theta, k, st) ? st[0] - dh : Double.NaN;
            if (Double.isNaN(fm) || fm < 0) {
                lo = m; fLo = fm;
                if (side == -1 && !Double.isNaN(fLo)) fHi *= 0.5;
                side = -1;
            } else {
                hi = m; fHi = fm;
                if (side == 1 && !Double.isNaN(fLo)) fLo *= 0.5;
                side = 1;
            }
            if (Math.abs(fm) < 0.005) return m;
        }
        return 0.5 * (lo + hi);
    }

    /** Closed-form vacuum speed. NaN if the target is above the launch line. */
    public static double vacuumSpeed(double d, double dh, double theta) {
        double c = Math.cos(theta);
        double denom = 2 * c * c * (d * Math.tan(theta) - dh);
        if (d <= 0 || denom <= 0) return Double.NaN;
        return Math.sqrt(G * d * d / denom);
    }

    /**
     * RK4-integrate a shot from (0,0) until it reaches horizontal distance d.
     * out = {y at d, vx, vy, time}. Returns false if it hits the floor or times out first.
     */
    static boolean flyTo(double d, double v, double theta, double k, double[] out) {
        double x = 0, y = 0, vx = v * Math.cos(theta), vy = v * Math.sin(theta), t = 0;
        double floor = -RobotConfig.LAUNCHER_EXIT_HEIGHT_IN;
        while (t < MAX_FLIGHT_SEC) {
            double px = x, py = y, pvx = vx, pvy = vy;

            double s1 = Math.hypot(vx, vy);
            double ax1 = -k * s1 * vx, ay1 = -G - k * s1 * vy;
            double vx2 = vx + ax1 * DT / 2, vy2 = vy + ay1 * DT / 2;
            double s2 = Math.hypot(vx2, vy2);
            double ax2 = -k * s2 * vx2, ay2 = -G - k * s2 * vy2;
            double vx3 = vx + ax2 * DT / 2, vy3 = vy + ay2 * DT / 2;
            double s3 = Math.hypot(vx3, vy3);
            double ax3 = -k * s3 * vx3, ay3 = -G - k * s3 * vy3;
            double vx4 = vx + ax3 * DT, vy4 = vy + ay3 * DT;
            double s4 = Math.hypot(vx4, vy4);
            double ax4 = -k * s4 * vx4, ay4 = -G - k * s4 * vy4;

            x += DT / 6 * (vx + 2 * vx2 + 2 * vx3 + vx4);
            y += DT / 6 * (vy + 2 * vy2 + 2 * vy3 + vy4);
            vx += DT / 6 * (ax1 + 2 * ax2 + 2 * ax3 + ax4);
            vy += DT / 6 * (ay1 + 2 * ay2 + 2 * ay3 + ay4);
            t += DT;

            if (x >= d) {
                double w = (d - px) / (x - px);
                out[0] = py + w * (y - py);
                out[1] = pvx + w * (vx - pvx);
                out[2] = pvy + w * (vy - pvy);
                out[3] = t - DT + w * DT;
                return true;
            }
            if (y < floor && vy < 0) return false;
        }
        return false;
    }

    // ---- Unit conversions ----

    /** Drag constant k (1/in) for a given drag coefficient. */
    public static double dragK(double cd) {
        double r = RobotConfig.BALL_DIAMETER_IN / 2;
        double area = Math.PI * r * r;
        return 0.5 * AIR_DENSITY_KG_PER_IN3 * cd * area / (RobotConfig.BALL_MASS_G / 1000.0);
    }

    public static double exitSpeedToTicks(double exitSpeed, double efficiency) {
        double wheelRevsPerSec = exitSpeed / efficiency / (Math.PI * RobotConfig.FLYWHEEL_DIAMETER_IN);
        return wheelRevsPerSec / RobotConfig.FLYWHEEL_REVS_PER_MOTOR_REV * RobotConfig.LAUNCHER_TICKS_PER_REV;
    }

    public static double ticksToExitSpeed(double ticksPerSec, double efficiency) {
        double wheelRevsPerSec = ticksPerSec / RobotConfig.LAUNCHER_TICKS_PER_REV * RobotConfig.FLYWHEEL_REVS_PER_MOTOR_REV;
        return wheelRevsPerSec * Math.PI * RobotConfig.FLYWHEEL_DIAMETER_IN * efficiency;
    }

    /** Vacuum fallback while the table builds: steepest-entry closed form, clamped to the hood range. */
    private Solution solveVacuumFallback(double d) {
        // tan(theta) = 2 dh / d - tan(entry) gives exactly the requested entry angle in vacuum.
        double angle = Math.toDegrees(Math.atan(2 * dh / d - Math.tan(Math.toRadians(maxEntry - 15))));
        angle = Math.max(minAngle, Math.min(maxAngle, angle));
        double theta = Math.toRadians(angle);
        double v = vacuumSpeed(d, dh, theta);
        if (Double.isNaN(v)) return Solution.invalid(d, "unreachable");
        double tof = d / (v * Math.cos(theta));
        double entry = Math.toDegrees(Math.atan2(v * Math.sin(theta) - G * tof, v * Math.cos(theta)));
        if (entry > maxEntry) return Solution.invalid(d, "too close (ball still rising)");
        return new Solution(true, "ok (vacuum)", d, angle, v, tof, entry, Double.NaN);
    }

    // ---- Calibration ----

    /** One shot that scored dead center: where it was taken from and what the launcher was doing. */
    public static final class GoodShot {
        public final double distanceIn, ticksPerSec, angleDeg;

        public GoodShot(double distanceIn, double ticksPerSec, double angleDeg) {
            this.distanceIn = distanceIn;
            this.ticksPerSec = ticksPerSec;
            this.angleDeg = angleDeg;
        }
    }

    /** Fit result: {efficiency, dragCd, relative spread of efficiencies (lower = model fits better)}. */
    public static double[] fitEfficiencyAndDrag(List<GoodShot> shots, double currentCd) {
        if (shots.isEmpty()) return new double[]{RobotConfig.LAUNCH_EFFICIENCY, currentCd, Double.NaN};
        double minD = Double.MAX_VALUE, maxD = 0;
        for (GoodShot s : shots) { minD = Math.min(minD, s.distanceIn); maxD = Math.max(maxD, s.distanceIn); }
        boolean fitDrag = shots.size() >= 2 && maxD - minD >= 24;

        double bestCd = currentCd, bestEff = Double.NaN, bestSpread = Double.MAX_VALUE;
        for (double cd = fitDrag ? 0 : currentCd; cd <= (fitDrag ? 1.5 : currentCd) + 1e-9; cd += 0.02) {
            double[] eff = efficiencies(shots, dragK(cd));
            if (eff == null) continue;
            double mean = 0;
            for (double e : eff) mean += e;
            mean /= eff.length;
            double var = 0;
            for (double e : eff) var += (e - mean) * (e - mean);
            double spread = Math.sqrt(var / eff.length) / mean;
            if (spread < bestSpread) { bestSpread = spread; bestCd = cd; bestEff = mean; }
        }
        return new double[]{bestEff, bestCd, bestSpread};
    }

    private static double[] efficiencies(List<GoodShot> shots, double k) {
        double dh = RobotConfig.HIVE_TARGET_HEIGHT_IN - RobotConfig.LAUNCHER_EXIT_HEIGHT_IN;
        double[] out = new double[shots.size()];
        for (int i = 0; i < out.length; i++) {
            GoodShot s = shots.get(i);
            double v = speedFor(s.distanceIn, dh, Math.toRadians(s.angleDeg), k, 5000);
            if (Double.isNaN(v)) return null;
            out[i] = v / ticksToExitSpeed(s.ticksPerSec, 1.0);
        }
        return out;
    }

    private static double lerp(double a, double b, double w) {
        return a + (b - a) * w;
    }
}
