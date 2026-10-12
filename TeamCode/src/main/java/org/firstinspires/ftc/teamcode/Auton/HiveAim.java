package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Autonomous alignment and shot calculation subsystem for scoring into the HIVE cell opening.
 * Uses Limelight 3A hardware-native 3D pose, PD control filtering, and PedroPathing Follower drivetrain manual control.
 *
 * All camera installation offsets, camera-to-shooter transforms, and target cluster offsets
 * are configured directly in the Limelight Web UI hardware pipeline.
 */
public class HiveAim {

    // Subsystem hardware objects
    private final Follower follower;
    private final Limelight3A limelight;

    // Vision pipeline index configured on Limelight Web UI for AprilTag cluster detection
    public static final int TAG_PIPELINE = 1;

    // Physical constants for parabolic motion kinematics
    private static final double INCHES_TO_METERS = 0.0254; // Conversion factor for SI physics calculations
    private static final double GRAVITY = 9.80665;        // Standard acceleration due to gravity (m/s^2)

    // Flywheel mechanical specifications
    private static final double FLYWHEEL_DIAMETER_M = 0.072; // Flywheel outer wheel diameter (meters)
    private static final double MAX_MOTOR_RPM = 4800.0;     // Maximum allowable motor RPM ceiling

    // Efficiency calibration factor to compensate for mechanical wheel slip and ball compression
    public static double BALL_SPEED_FACTOR = 0.75;

    // Flywheel gear ratio (flywheel revolutions per motor revolution)
    public static double WHEEL_REVS_PER_MOTOR_REV = 1.0;

    // Allowable launch angle bounds for iteration loop (degrees)
    public static double MIN_LAUNCH_ANGLE = 30.0;
    public static double MAX_LAUNCH_ANGLE = 75.0;

    // Shooter exit height above the floor (inches)
    public static double SHOOTER_EXIT_HEIGHT_IN = 12.0;

    // Direction scalar (+1.0 or -1.0) to flip turning direction if drivetrain motors or gearboxes reverse rotation
    public static double TURN_SIGN = 1.0;

    // Proportional (kP) and Derivative (kD) gains for heading alignment PD loop
    private static final double ALIGN_KP = 0.014;
    private static final double ALIGN_KD = 0.002;

    // Power output caps and anti-stall minimum power floor
    private static final double MAX_TURN = 0.22; // Cap turning power at 22% to prevent overshoot
    private static final double MIN_TURN = 0.055; // Minimum 5.5% power floor to overcome static wheel friction

    // Lock parameters
    private static final double ALIGN_TOLERANCE = 1.5; // Alignment complete deadband (±1.5 degrees)
    private static final double LOCK_TIME = 0.15;       // Continuous lock time required (150ms)
    private static final double ALIGN_TIMEOUT = 2.5;    // Maximum alignment timeout guard (2.5 seconds)
    private static final double PIPELINE_SETTLE = 0.25;  // Vision pipeline switch settle delay (250ms)

    // Enforces descending ball trajectory into top cell opening (minimum downward vertical velocity)
    private static final double MIN_DESCENT_SPEED = 0.30; // 0.30 m/s downward velocity

    // Timers
    private final ElapsedTime alignTimer = new ElapsedTime();
    private final ElapsedTime lockTimer = new ElapsedTime();
    private boolean lockStarted = false;
    private boolean aligned = false;

    // State & Output variables
    public int visibleTags = 0;

    public double yawErrorDeg = Double.NaN;
    public double distanceIn = Double.NaN;
    public double targetHeightIn = Double.NaN;

    public double launchAngleDeg = Double.NaN;
    public double motorRPM = Double.NaN;
    public double flightTime = Double.NaN;

    private double lastErrorDeg = Double.NaN;
    private double lastErrorTime = 0;

    public String status = "Idle";
    public String stage = "";

    /**
     * Initializes HiveAim with PedroPathing Follower and Limelight 3A camera.
     */
    public HiveAim(
            Follower follower,
            Limelight3A limelight) {

        this.follower = follower;
        this.limelight = limelight;
    }

    /**
     * Updates target geometry directly from Limelight hardware 3D pose (botpose).
     * Camera-to-robot transforms and target offsets are handled in Limelight Web UI hardware config.
     *
     * @return True if valid 3D target geometry was updated.
     */
    private boolean updateTargetGeometry() {

        LLResult result = limelight.getLatestResult();

        // Ensure valid vision result and correct pipeline index
        if (result == null
                || !result.isValid()
                || result.getPipelineIndex() != TAG_PIPELINE) {
            visibleTags = 0;
            return false;
        }

        visibleTags = (result.getFiducialResults() != null)
                ? result.getFiducialResults().size()
                : 1;

        // Limelight handles 3D pose and camera offsets natively in hardware
        Pose3D pose = result.getBotpose();

        if (pose == null || pose.getPosition() == null) {
            return false;
        }

        Position p = pose.getPosition().toUnit(
                DistanceUnit.INCH
        );

        double targetForward = p.z; // Z = forward distance to target
        double targetRight = p.x;   // X = lateral right offset to target

        if (!Double.isFinite(targetForward)
                || !Double.isFinite(targetRight)
                || targetForward <= 0) {
            return false;
        }

        // Distance = hypotenuse of forward and lateral offsets
        distanceIn = Math.hypot(
                targetForward,
                targetRight
        );

        // Yaw error = angle offset in degrees to center target in front of robot
        yawErrorDeg = Math.toDegrees(
                Math.atan2(targetRight, targetForward)
        );

        targetHeightIn = p.y; // Y = vertical target height relative to camera

        return Double.isFinite(yawErrorDeg);
    }

    /**
     * Calculates required flywheel motor RPM and launch angle using parabolic trajectory physics.
     * Solves v0^2 = (g * x^2) / [2 * cos^2(theta) * (x * tan(theta) - deltaH)]
     * and filters for trajectories where the ball enters the top opening on a downward descent path.
     */
    private void calculateShot() {

        launchAngleDeg = Double.NaN;
        motorRPM = Double.NaN;
        flightTime = Double.NaN;

        if (!Double.isFinite(targetHeightIn)
                || !Double.isFinite(SHOOTER_EXIT_HEIGHT_IN)
                || BALL_SPEED_FACTOR <= 0
                || WHEEL_REVS_PER_MOTOR_REV <= 0) {

            status = "Aligned; shooting geometry missing";
            return;
        }

        double x = distanceIn * INCHES_TO_METERS;

        double deltaH =
                (targetHeightIn - SHOOTER_EXIT_HEIGHT_IN)
                        * INCHES_TO_METERS;

        // Iterate through launch angles to find the optimal descending trajectory
        for (double angle = MIN_LAUNCH_ANGLE;
             angle <= MAX_LAUNCH_ANGLE;
             angle += 0.25) {

            double theta = Math.toRadians(angle);

            double cos = Math.cos(theta);
            double tan = Math.tan(theta);

            double denominator =
                    2.0 * cos * cos
                            * (x * tan - deltaH);

            if (denominator <= 0) {
                continue; // Angle too shallow to reach target height
            }

            double velocitySquared =
                    GRAVITY * x * x / denominator;

            if (!Double.isFinite(velocitySquared)
                    || velocitySquared <= 0) {
                continue;
            }

            double velocity = Math.sqrt(
                    velocitySquared
            );

            double time = x / (velocity * cos);

            // Compute vertical velocity at target entry: vy = v0 * sin(theta) - g * t
            double verticalVelocity =
                    velocity * Math.sin(theta)
                            - GRAVITY * time;

            // Require ball to be descending (vy < -0.30 m/s) to drop cleanly into cell opening
            if (verticalVelocity > -MIN_DESCENT_SPEED) {
                continue;
            }

            // Convert linear velocity (m/s) to flywheel RPM, accounting for slip factor
            double wheelRPM =
                    velocity * 60.0
                            / (Math.PI
                            * FLYWHEEL_DIAMETER_M
                            * BALL_SPEED_FACTOR);

            double requiredMotorRPM =
                    wheelRPM / WHEEL_REVS_PER_MOTOR_REV;

            // Validate motor RPM bounds
            if (!Double.isFinite(requiredMotorRPM)
                    || requiredMotorRPM <= 0
                    || requiredMotorRPM > MAX_MOTOR_RPM) {
                continue;
            }

            // Select trajectory that minimizes required motor RPM
            if (!Double.isFinite(motorRPM)
                    || requiredMotorRPM < motorRPM) {

                motorRPM = requiredMotorRPM;
                launchAngleDeg = angle;
                flightTime = time;
            }
        }

        status = Double.isFinite(motorRPM)
                ? "Aligned; shot calculated"
                : "Aligned; no feasible descending shot";
    }

    /**
     * Clamps a value between minimum and maximum bounds.
     */
    private double clamp(
            double value,
            double minimum,
            double maximum) {

        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }

    /**
     * Builds PedroPathing Command for automated vision alignment and shot physics calculation.
     *
     * @param stageName     TeleOp/Auton step identifier for logging.
     * @param pipelineAfter Pipeline index to switch Limelight back to when complete.
     * @return Command object for PedroPathing Scheduler.
     */
    public Command alignAndCalculate(
            String stageName,
            int pipelineAfter) {

        return Command.build()

                .setStart(() -> {

                    stage = stageName;
                    status = "Switching to AprilTags";

                    aligned = false;
                    lockStarted = false;

                    visibleTags = 0;

                    yawErrorDeg = Double.NaN;
                    distanceIn = Double.NaN;
                    targetHeightIn = Double.NaN;

                    launchAngleDeg = Double.NaN;
                    motorRPM = Double.NaN;
                    flightTime = Double.NaN;

                    lastErrorDeg = Double.NaN;
                    lastErrorTime = 0;

                    alignTimer.reset();

                    limelight.pipelineSwitch(TAG_PIPELINE);
                    follower.manual(0, 0, 0);
                })

                .setExecute(() -> {

                    // Allow pipeline switch to settle before processing vision frames
                    if (alignTimer.seconds()
                            < PIPELINE_SETTLE) {

                        follower.manual(0, 0, 0);
                        return;
                    }

                    // Update 3D geometry from Limelight
                    if (!updateTargetGeometry()) {

                        status = "Waiting for HIVE tags (" + visibleTags + " seen)";
                        lockStarted = false;

                        follower.manual(0, 0, 0);
                        return;
                    }

                    // Calculate PD error and derivative rate of change
                    double currentTime = alignTimer.seconds();
                    double dt = currentTime - lastErrorTime;

                    double error = yawErrorDeg;
                    double derivative = 0;

                    if (dt > 0.000001 && !Double.isNaN(lastErrorDeg)) {
                        derivative = (error - lastErrorDeg) / dt;
                    }

                    lastErrorDeg = error;
                    lastErrorTime = currentTime;

                    // Check if alignment is within tolerance deadband (±1.5 degrees)
                    if (Math.abs(error)
                            <= ALIGN_TOLERANCE) {

                        follower.manual(0, 0, 0);

                        if (!lockStarted) {
                            lockStarted = true;
                            lockTimer.reset();
                        }

                        // Require continuous lock for 150ms before calculating shot
                        if (lockTimer.seconds() >= LOCK_TIME) {

                            aligned = true;
                            calculateShot();
                        }

                        return;
                    }

                    lockStarted = false;
                    status = "Aligning to HIVE";

                    // PD turning output calculation: turn = TURN_SIGN * (kP * error + kD * derivative)
                    double turn =
                            TURN_SIGN * ((ALIGN_KP * error) + (ALIGN_KD * derivative));

                    // Enforce minimum power floor (5.5%) to overcome static motor friction
                    if (Math.abs(turn) < MIN_TURN) {
                        turn = Math.copySign(
                                MIN_TURN,
                                turn
                        );
                    }

                    // Clamp maximum turning power to 22% to prevent overshooting
                    turn = clamp(
                            turn,
                            -MAX_TURN,
                            MAX_TURN
                    );

                    follower.manual(0, 0, turn);
                })

                .setDone(() ->
                        aligned ||
                                alignTimer.seconds() >= ALIGN_TIMEOUT
                )

                .setEnd(endCondition -> {

                    follower.manual(0, 0, 0);

                    if (!aligned) {

                        status = "Alignment failed or timed out";

                        launchAngleDeg = Double.NaN;
                        motorRPM = Double.NaN;
                    }

                    limelight.pipelineSwitch(pipelineAfter);
                });
    }

    /**
     * Outputs diagnostic telemetry to Driver Station.
     */
    public void addTelemetry(Telemetry telemetry) {

        telemetry.addData("Shoot stage", stage);
        telemetry.addData("Aim status", status);

        telemetry.addData("Visible HIVE tags", visibleTags);

        telemetry.addData(
                "Yaw error (deg)",
                yawErrorDeg
        );

        telemetry.addData(
                "Opening distance (in)",
                distanceIn
        );

        telemetry.addData(
                "Opening height (in)",
                targetHeightIn
        );

        telemetry.addData(
                "Launch angle (deg)",
                launchAngleDeg
        );

        telemetry.addData(
                "Required motor RPM",
                motorRPM
        );

        telemetry.addData(
                "Flight time (sec)",
                flightTime
        );
    }
}
