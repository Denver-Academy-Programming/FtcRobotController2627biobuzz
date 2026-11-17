package org.firstinspires.ftc.teamcode.FullBot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/*
 * The whole robot. OpModes call update() exactly once per loop; it does one bulk read of every motor
 * (fast, consistent loop times) and steps every subsystem in the right order:
 *   odometry -> vision (needs the fresh pose) -> shooter logic -> hood/flywheel/intake outputs.
 */
public class Robot {

    // Carries alliance + pose from AUTO into TeleOp (static survives between OpModes).
    public static Pose lastPose = null;
    public static Boolean lastRedAlliance = null;

    public final Drivetrain drive;
    public final Flywheel flywheel;
    public final Hood hood;
    public final Intake intake;
    public final HiveVision vision;
    public final ShotSolver solver;
    public final Shooter shooter;

    private final List<LynxModule> hubs;
    private final VoltageSensor battery;
    private double volts = 12.5;
    private double voltTimer = 0;

    private long lastNanos = 0;
    private double dt = 0.02;
    private double intakeRequest = 0;

    public Robot(HardwareMap hw) {
        Hw.reset();
        hubs = hw.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        battery = hw.voltageSensor.iterator().hasNext() ? hw.voltageSensor.iterator().next() : null;

        drive = new Drivetrain(hw);
        flywheel = new Flywheel(hw);
        hood = new Hood(hw);
        intake = new Intake(hw);
        vision = new HiveVision();
        vision.init(hw);
        solver = new ShotSolver(RobotConfig.BALL_DRAG_CD, hood.present());
        solver.buildTableAsync();
        shooter = new Shooter(flywheel, hood, vision, solver);
        volts = readVolts();
    }

    private double readVolts() {
        if (battery == null) return 12.5;
        double v = battery.getVoltage();
        return v > 1 ? v : 12.5;   // 0 when the hub isn't reporting: assume a normal battery
    }

    /** Driver / auto intake request; the shooter overrides it while feeding. */
    public void setIntake(double power) { intakeRequest = power; }

    public void update() {
        long now = System.nanoTime();
        dt = lastNanos == 0 ? 0.02 : Math.min(0.1, (now - lastNanos) / 1e9);
        lastNanos = now;

        for (LynxModule hub : hubs) hub.clearBulkCache();

        // The voltage read isn't part of the bulk read, so only refresh it a few times a second.
        voltTimer += dt;
        if (voltTimer > 0.25) {
            volts += 0.5 * (readVolts() - volts);
            voltTimer = 0;
        }

        drive.update();
        vision.update(drive);
        shooter.update(drive, dt);
        hood.update(dt);
        flywheel.update(dt, volts);
        intake.set(shooter.feeding() ? RobotConfig.FEED_POWER : intakeRequest);
        intake.update(dt);
    }

    public double dt() { return dt; }
    public double volts() { return volts; }

    public void stop() {
        drive.stop();
        flywheel.stop();
        intake.stop();
        vision.close();
        lastPose = drive.pose();
        lastRedAlliance = vision.isRedAlliance();
    }

    public void addTelemetry(Telemetry t) {
        ShotSolver.Solution s = shooter.solution();
        String missing = Hw.missing();
        if (!missing.isEmpty()) t.addData("Missing (skipped)", missing);
        t.addData("Shooter", shooter.status(drive));
        t.addData("Target", vision.targetName());
        t.addData("Tags seen", "%s   (tag height %.1f in)", vision.seenIds(), vision.lastTagHeight());
        if (s != null && s.valid) {
            t.addData("Shot", "%.1f in  hood %.1f°  exit %.0f in/s  tof %.2fs  entry %.0f°  ±%.1f in",
                    shooter.distance(), s.angleDeg, s.exitSpeed, s.timeOfFlight, s.entryAngleDeg, s.expectedMissIn);
        }
        if (!Double.isNaN(shooter.aimHeading())) {
            t.addData("Aim", "err %.2f°  tol %.2f°",
                    Math.toDegrees(drive.headingError(shooter.aimHeading())), shooter.headingToleranceDeg());
        }
        t.addData("Flywheel", "L %.0f  R %.0f  / %.0f  %s  (recovery %.2fs)",
                flywheel.velocityLeft(), flywheel.velocityRight(), flywheel.target(),
                flywheel.ready() ? "AT SPEED" : "", flywheel.lastRecoverySec());
        if (hood.present()) t.addData("Hood", "%.1f° -> %.1f° %s", hood.angleDeg(), hood.targetAngleDeg(), hood.settled() ? "" : "moving");
        else t.addData("Launch angle", "%.1f° (fixed, no hood)", hood.angleDeg());
        t.addData("Pose", drive.pose().toString());
        t.addData("Battery / loop", "%.2f V   %.0f ms %s", volts, dt * 1000, solver.tableReady() ? "" : "(solver table building)");
    }
}
