package org.firstinspires.ftc.teamcode.biobuzz;

/** Immutable 2D pose: x, y in inches, heading in radians (CCW positive). */
public final class Pose {
    public final double x, y, heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /** Pose built with heading in degrees, handy for writing autonomous waypoints. */
    public static Pose deg(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    /** Field position of a point given in this pose's robot frame (forward, left). */
    public double[] toField(double forward, double left) {
        double c = Math.cos(heading), s = Math.sin(heading);
        return new double[]{x + forward * c - left * s, y + forward * s + left * c};
    }

    /** Mirror across the field's x axis (swap alliance side). */
    public Pose mirrored() {
        return new Pose(x, -y, -heading);
    }

    public double distanceTo(Pose o) {
        return Math.hypot(o.x - x, o.y - y);
    }

    /** Wrap an angle to (-pi, pi]. */
    public static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    @Override
    public String toString() {
        return String.format("(%.1f, %.1f, %.1f°)", x, y, Math.toDegrees(heading));
    }
}
