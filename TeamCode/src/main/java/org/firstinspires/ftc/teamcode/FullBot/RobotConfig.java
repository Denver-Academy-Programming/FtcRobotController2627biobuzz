package org.firstinspires.ftc.teamcode.biobuzz;

/*
 * Every number that depends on the physical robot lives here, so tuning never means hunting through code.
 *
 * Units: inches, seconds, degrees (unless a name says otherwise). Encoder velocities are ticks/second.
 *
 * Robot frame:  +forward = intake side, +left, heading measured counter-clockwise (CCW) from above.
 * Field frame:  odometry frame set at the start of AUTO (+x = robot forward at start, +y = robot left at start).
 *
 * Driver Station configuration names:
 *   Motors:  Front_Left, Front_Right, Rear_Left, Rear_Right, launcher_left, launcher_right
 *   intake:  DC motor OR Continuous Rotation Servo (roller intake + feeder), either works
 *   Servo:   hood          (sets the launch angle; only used when HAS_HOOD = true)
 *   Webcam:  Webcam 1
 *   IMU:     imu           (Control Hub built-in)
 * Every device is optional: anything missing from the configuration is skipped and listed as "Missing" in telemetry.
 *
 * Fields that are "public static" (not final) can be changed live by the tuning OpMode; copy the tuned values back here.
 *
 * Value tags:
 *   [SOURCE]       taken from the Competition Manual / part datasheet. Leave alone.
 *   [PLACEHOLDER]  a guess so the code runs. MEASURE it on the robot, or TUNE it, before trusting the shooter.
 *   (no tag)       sensible default, adjust only if needed.
 */
public final class RobotConfig {
    private RobotConfig() {}

    // =====================================================================================
    // DRIVETRAIN
    // =====================================================================================
    // [PLACEHOLDER] assumes goBILDA 5203 312 RPM motors (537.7 ticks/rev) and 104 mm mecanum wheels. Check your parts.
    public static final double DRIVE_TICKS_PER_REV = 537.7;
    public static final double WHEEL_DIAMETER_IN = 104.0 / 25.4;
    public static final double DRIVE_TICKS_PER_INCH = DRIVE_TICKS_PER_REV / (WHEEL_DIAMETER_IN * Math.PI);
    // [PLACEHOLDER] TUNE: mecanum rollers slip sideways. Strafe exactly 48 in by tape measure, divide 48 by what telemetry says.
    public static double STRAFE_MULTIPLIER = 1.10;
    // [PLACEHOLDER] TUNE: flip these if pushing the stick forward doesn't drive forward.
    public static final boolean LEFT_DRIVE_REVERSED = true;
    public static final boolean RIGHT_DRIVE_REVERSED = false;
    // TeleOp only: flips which end of the robot the left stick treats as "front" (forward/back AND left/right).
    // Set true because trials showed stick-forward drove backward and stick-left strafed right. Turning is unaffected.
    public static boolean TELEOP_DRIVE_INVERTED = true;

    // [PLACEHOLDER] MEASURE: Control Hub mounting (see ConceptExploringIMUOrientation if unsure).
    public static final String HUB_LOGO_DIRECTION = "UP";        // UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD
    public static final String HUB_USB_DIRECTION = "FORWARD";

    // [PLACEHOLDER] TUNE: heading controller (output = turn power, error in radians).
    // D uses the IMU's gyro rate directly (no noise from differentiating).
    public static double HEADING_KP = 1.6;
    public static double HEADING_KD = 0.12;
    public static double HEADING_KS = 0.05;      // minimum power that actually moves the robot
    public static double HEADING_MAX_POWER = 0.7;

    // [PLACEHOLDER] TUNE: point-to-point (autonomous). Error in inches -> drive power.
    public static double TRANSLATE_KP = 0.06;
    public static double TRANSLATE_KS = 0.06;
    public static double TRANSLATE_MAX_ACCEL = 2.5;  // power units per second (slew limit, stops wheelies + slip)
    public static double POSITION_TOLERANCE_IN = 1.0;
    public static double HEADING_TOLERANCE_DEG = 2.0;

    // =====================================================================================
    // LAUNCHER GEOMETRY
    // =====================================================================================
    // [PLACEHOLDER] MEASURE: where the POLLEN leaves the flywheel, relative to the robot's center of rotation, from above.
    public static final double LAUNCHER_FORWARD_IN = -5.0;
    public static final double LAUNCHER_LEFT_IN = 0.0;
    // [PLACEHOLDER] MEASURE: floor to exit point. Higher = closer minimum shooting distance.
    public static final double LAUNCHER_EXIT_HEIGHT_IN = 14.0;
    // [PLACEHOLDER] MEASURE: direction the launcher shoots, relative to robot forward. 0 = over the intake, 180 = out the back.
    public static final double LAUNCHER_YAW_DEG = 0.0;

    // Flywheels: two motors, one per side, each on its own shaft with its own velocity loop.
    // [PLACEHOLDER] MEASURE: flywheel wheel diameter.
    public static final double FLYWHEEL_DIAMETER_IN = 4.0;
    public static final double LAUNCHER_TICKS_PER_REV = 28.0;          // [SOURCE] bare goBILDA 6000 RPM motor encoder
    // [PLACEHOLDER] MEASURE: flywheel revs per motor rev (>1 if geared up, 1.0 for direct drive).
    public static final double FLYWHEEL_REVS_PER_MOTOR_REV = 1.0;
    public static final double LAUNCHER_MAX_TICKS_PER_SEC = 2600;      // stay under free speed (2800) so there's headroom to recover
    // [PLACEHOLDER] TUNE: both wheels must spin so they throw the ball out. Flip one if they fight.
    public static final boolean LAUNCHER_LEFT_REVERSED = false;
    public static final boolean LAUNCHER_RIGHT_REVERSED = true;

    // Adjustable hood? false = launch angle is fixed at FIXED_LAUNCH_ANGLE_DEG and the solver only picks flywheel speed.
    public static final boolean HAS_HOOD = false;
    // [PLACEHOLDER] MEASURE (phone inclinometer on the exit ramp): launch angle above horizontal with no hood.
    // Steeper = can shoot from closer, but needs more speed far away.
    public static double FIXED_LAUNCH_ANGLE_DEG = 50.0;

    // Hood servo (only when HAS_HOOD). Calibrate with ShooterTuner "raw servo" mode + a phone inclinometer app:
    // put the servo at two positions, measure the actual exit angle at each, enter them here.
    // [PLACEHOLDER] MEASURE: the two calibration points.
    public static double HOOD_CAL_POS_A = 0.20, HOOD_CAL_ANGLE_A_DEG = 30.0;
    public static double HOOD_CAL_POS_B = 0.80, HOOD_CAL_ANGLE_B_DEG = 65.0;
    // [PLACEHOLDER] MEASURE: mechanical limits of the hood. A steeper max angle allows closer shots.
    public static double HOOD_MIN_ANGLE_DEG = 30.0;
    public static double HOOD_MAX_ANGLE_DEG = 65.0;
    // [PLACEHOLDER] MEASURE: time a full 0->1 servo sweep; HOOD_SERVO_SPEED = 1 / that time.
    public static double HOOD_SERVO_SPEED = 1.6;       // servo position units per second (for "has it arrived yet")
    public static double HOOD_SETTLE_SEC = 0.06;       // extra time after arriving before we trust it

    // =====================================================================================
    // FLYWHEEL CONTROL  (tune with ShooterTuner)
    // =====================================================================================
    // power = (kS + kV * target) * 12V / battery + kP * error;   full power when far below target (bang-bang recovery)
    // [PLACEHOLDER] TUNE all gains below with ShooterTuner.
    public static double FLY_KS = 0.03;
    public static double FLY_KV = 1.0 / 2750.0;        // ShooterTuner prints a measured value: copy it here
    public static double FLY_KP = 0.0012;
    public static double FLY_RECOVERY_BAND = 120;      // ticks/s below target -> full power
    public static double FLY_READY_TOLERANCE = 25;     // ticks/s
    public static double FLY_READY_STABLE_SEC = 0.08;  // must be inside tolerance this long
    public static double FLY_MAX_SIDE_MISMATCH = 30;   // left vs right ticks/s; mismatch puts side-spin on the ball
    public static double FLY_SHOT_DROP = 70;           // [PLACEHOLDER] TUNE: watch the dip when a ball goes through
    public static double FLY_VELOCITY_FILTER = 0.5;    // 0..1, higher = less filtering

    // =====================================================================================
    // INTAKE / FEED  (intake also pushes POLLEN into the flywheel)
    // =====================================================================================
    public static double INTAKE_POWER = 1.0;
    public static double FEED_POWER = 0.8;             // [PLACEHOLDER] TUNE: slow enough that balls don't double-feed
    public static double MIN_SHOT_SPACING_SEC = 0.12;  // after a shot is detected, stop feeding at least this long
    // [PLACEHOLDER] TUNE: flip if the intake spins outward when pulling the right trigger.
    public static final boolean INTAKE_REVERSED = false;

    // =====================================================================================
    // BALL + HIVE  (projectile model)
    // =====================================================================================
    // [SOURCE] Competition Manual TU04 Fig 9-10: raised CELL opening spans 53.5 in to 65.6 in above the tiles.
    public static final double HIVE_TARGET_HEIGHT_IN = (53.5 + 65.6) / 2;
    // [PLACEHOLDER] MEASURE: half-width of the opening you're willing to miss by. Sets the heading tolerance at each distance.
    public static double HIVE_AIM_HALF_WIDTH_IN = 2.5;
    // The CELL faces up, so POLLEN must be coming DOWN at least this steeply to drop in.
    public static double MAX_ENTRY_ANGLE_DEG = -20.0;

    // POLLEN: [SOURCE] Competition Manual TU06 9.8: "approximately 2.8 in. (7.1 cm) Gopher ResisDent polyethylene
    // balls in yellow (am-5851_yellow)". Not perfectly spherical and may vary in size.
    // Mass: [SOURCE] retailer listing (Plex / Hardware Depot) 0.055 lb = 24.9 g. The manual doesn't list a weight.
    //   Weigh a few of your own on a kitchen scale and use the average.
    // NECTAR is ~3.6 in (9.1 cm), same material. If you launch NECTAR, it needs its own diameter/mass/efficiency.
    public static final double BALL_DIAMETER_IN = 2.8;
    public static final double BALL_MASS_G = 24.9;
    // Air drag. Smooth polyethylene sphere at our speeds (Reynolds ~3e4..5e4) -> Cd ~0.47.
    // ShooterTuner fits Cd from good shots at 2+ distances. Set 0 for pure vacuum projectile equations.
    public static double BALL_DRAG_CD = 0.47;

    // [PLACEHOLDER] TUNE: exit speed / flywheel surface speed. Calibrate with ShooterTuner (record good shots).
    public static double LAUNCH_EFFICIENCY = 0.50;

    // How much the actuators wander, used to pick the hood angle that is LEAST sensitive to errors.
    public static double SPEED_NOISE_FRACTION = 0.015;   // 1.5% exit-speed variation shot to shot
    public static double ANGLE_NOISE_DEG = 0.5;

    // Range of distances the solver pre-computes (launcher exit to CELL center, horizontal).
    public static final double SOLVER_MIN_DISTANCE_IN = 18.0;
    public static final double SOLVER_MAX_DISTANCE_IN = 200.0;

    // Shot on the move: lead the target by robot velocity x time of flight.
    public static boolean SHOOT_ON_MOVE = true;
    public static double MAX_AIM_TURN_RATE_DEG_PER_SEC = 25; // don't fire while still spinning faster than this

    // =====================================================================================
    // CAMERA
    // =====================================================================================
    // [PLACEHOLDER] MEASURE: lens position from the robot center, height above the tiles, and tilt.
    public static final double CAMERA_FORWARD_IN = -4.0;
    public static final double CAMERA_LEFT_IN = 0.0;
    public static final double CAMERA_HEIGHT_IN = 10.0;
    public static final double CAMERA_PITCH_UP_DEG = 30.0; // tags face DOWN from under the CELL, so tilt the camera up
    public static final double CAMERA_YAW_DEG = 0.0;       // 0 = looks forward, same convention as LAUNCHER_YAW_DEG
    public static final int CAMERA_EXPOSURE_MS = 6;        // short exposure = less motion blur
    public static final int CAMERA_GAIN = 250;

    // [PLACEHOLDER] TUNE: tags below this height belong to the lowered CELL (can't score).
    // Check "tag height" telemetry with each CELL up and set this halfway between the two readings.
    public static double UP_CELL_MIN_TAG_HEIGHT_IN = 42.0;
    // A new sighting farther than this from the remembered target means the HIVE tipped/moved: start over.
    public static double TARGET_RESET_JUMP_IN = 8.0;
    public static double TARGET_FILTER_ALPHA = 0.35;       // 0..1 per sighting, higher trusts the newest sighting more
    // Keep aiming with odometry this long after the last sighting.
    public static double TARGET_MEMORY_SEC = 4.0;
    // Constant trims if shots are consistently off once everything else is tuned.
    public static double AIM_TRIM_DEG = 0.0;          // + = aim further left (CCW)
    public static double DISTANCE_TRIM_IN = 0.0;      // + = shoot further
}
