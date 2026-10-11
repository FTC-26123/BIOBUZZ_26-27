
package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class HiveAim {

    private final Follower follower;
    private final Limelight3A limelight;

    public static final int TAG_PIPELINE = 1;

    private static final int FIRST_TAG = 34;
    private static final int LAST_TAG = 37;

    private static final double IN_TO_M = 0.0254;
    private static final double M_TO_IN = 39.37007874;
    private static final double GRAVITY = 9.80665;

    private static final double FLYWHEEL_DIAMETER_M = 0.072;
    private static final double MAX_MOTOR_RPM = 4800.0;

    // Calibrate using real ball velocity measurements.
    public static double BALL_SPEED_FACTOR = 0.75;

    // Flywheel revolutions per motor revolution.
    public static double WHEEL_REVS_PER_MOTOR_REV = 1.0;

    public static double MIN_LAUNCH_ANGLE = 30.0;
    public static double MAX_LAUNCH_ANGLE = 75.0;

    // Camera installation measurements.
    // Positive pitch means pointing downward.
    // Positive yaw means pointing to the robot's right.
    public static double CAMERA_PITCH_DOWN_DEG = 0.0;
    public static double CAMERA_YAW_RIGHT_DEG = 0.0;

    public static double CAMERA_HEIGHT_IN = Double.NaN;
    public static double SHOOTER_EXIT_HEIGHT_IN = Double.NaN;

    // Shooter exit relative to camera lens.
    public static double SHOOTER_FORWARD_IN = 0.0;
    public static double SHOOTER_RIGHT_IN = 0.0;

    // Shooter's horizontal direction relative to
    // the robot's forward direction.
    public static double SHOOTER_YAW_RIGHT_DEG = 0.0;

    // Physical tag-cluster midpoint to CELL opening center.
    // Horizontal zeros assume the cluster is centered
    // under the opening; verify this on the field.
    public static double OPENING_FORWARD_IN = 0.0;
    public static double OPENING_RIGHT_IN = 0.0;
    public static double OPENING_UP_IN = Double.NaN;

    // Verify rotation direction with wheels elevated.
    // Positive error means target is to the right.
    public static double TURN_SIGN = 1.0;

    private static final double ALIGN_KP = 0.014;
    private static final double MAX_TURN = 0.22;
    private static final double MIN_TURN = 0.055;

    private static final double ALIGN_TOLERANCE = 1.5;
    private static final double LOCK_TIME = 0.15;
    private static final double ALIGN_TIMEOUT = 2.5;
    private static final double PIPELINE_SETTLE = 0.25;

    private static final double MIN_DESCENT_SPEED = 0.30;

    private final ElapsedTime alignTimer = new ElapsedTime();
    private final ElapsedTime lockTimer = new ElapsedTime();

    private boolean lockStarted = false;
    private boolean aligned = false;

    public int visibleTags = 0;

    public double yawErrorDeg = Double.NaN;
    public double distanceIn = Double.NaN;
    public double targetHeightIn = Double.NaN;

    public double launchAngleDeg = Double.NaN;
    public double motorRPM = Double.NaN;
    public double flightTime = Double.NaN;

    public String status = "Idle";
    public String stage = "";

    public HiveAim(
            Follower follower,
            Limelight3A limelight) {

        this.follower = follower;
        this.limelight = limelight;
    }

    private static class TagCenter {

        final double right;
        final double down;
        final double forward;

        TagCenter(double right, double down, double forward) {
            this.right = right;
            this.down = down;
            this.forward = forward;
        }
    }

    private TagCenter readFourTags() {

        visibleTags = 0;

        LLResult result = limelight.getLatestResult();

        if (result == null
                || !result.isValid()
                || result.getPipelineIndex() != TAG_PIPELINE) {
            return null;
        }

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        if (tags == null || tags.isEmpty()) {
            return null;
        }

        double[] right = new double[4];
        double[] down = new double[4];
        double[] forward = new double[4];

        boolean[] found = new boolean[4];

        for (LLResultTypes.FiducialResult tag : tags) {

            int id = tag.getFiducialId();

            if (id < FIRST_TAG || id > LAST_TAG) {
                continue;
            }

            int index = id - FIRST_TAG;

            if (found[index]) {
                continue;
            }

            Pose3D pose = tag.getTargetPoseCameraSpace();

            if (pose == null || pose.getPosition() == null) {
                continue;
            }

            Position p = pose.getPosition().toUnit(
                    DistanceUnit.METER
            );

            if (!Double.isFinite(p.x)
                    || !Double.isFinite(p.y)
                    || !Double.isFinite(p.z)
                    || p.z <= 0) {
                continue;
            }

            right[index] = p.x * M_TO_IN;
            down[index] = p.y * M_TO_IN;
            forward[index] = p.z * M_TO_IN;

            found[index] = true;
            visibleTags++;
        }

        if (visibleTags != 4) {
            return null;
        }

        double x = 0;
        double y = 0;
        double z = 0;

        for (int i = 0; i < 4; i++) {
            x += right[i];
            y += down[i];
            z += forward[i];
        }

        return new TagCenter(
                x / 4.0,
                y / 4.0,
                z / 4.0
        );
    }

    private boolean updateTargetGeometry() {

        TagCenter center = readFourTags();

        if (center == null) {
            return false;
        }

        double pitch = Math.toRadians(
                CAMERA_PITCH_DOWN_DEG
        );

        double yaw = Math.toRadians(
                CAMERA_YAW_RIGHT_DEG
        );

        // Limelight legacy camera coordinates:
        // X = right, Y = down, Z = forward.

        // Correct downward pitch.
        double forwardLevel =
                center.forward * Math.cos(pitch)
                        - center.down * Math.sin(pitch);

        double upFromCamera =
                -center.forward * Math.sin(pitch)
                        - center.down * Math.cos(pitch);

        // Correct camera yaw relative to robot.
        double rightRobot =
                center.right * Math.cos(yaw)
                        + forwardLevel * Math.sin(yaw);

        double forwardRobot =
                forwardLevel * Math.cos(yaw)
                        - center.right * Math.sin(yaw);

        // Convert tag midpoint to desired scoring point.
        double targetForward =
                forwardRobot
                        + OPENING_FORWARD_IN
                        - SHOOTER_FORWARD_IN;

        double targetRight =
                rightRobot
                        + OPENING_RIGHT_IN
                        - SHOOTER_RIGHT_IN;

        if (!Double.isFinite(targetForward)
                || !Double.isFinite(targetRight)
                || targetForward <= 0) {
            return false;
        }

        distanceIn = Math.hypot(
                targetForward,
                targetRight
        );

        yawErrorDeg = Math.toDegrees(
                Math.atan2(targetRight, targetForward)
        ) - SHOOTER_YAW_RIGHT_DEG;

        if (Double.isFinite(CAMERA_HEIGHT_IN)
                && Double.isFinite(OPENING_UP_IN)) {

            targetHeightIn =
                    CAMERA_HEIGHT_IN
                            + upFromCamera
                            + OPENING_UP_IN;

        } else {
            targetHeightIn = Double.NaN;
        }

        return Double.isFinite(yawErrorDeg);
    }

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

        double x = distanceIn * IN_TO_M;

        double deltaH =
                (targetHeightIn - SHOOTER_EXIT_HEIGHT_IN)
                        * IN_TO_M;

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
                continue;
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

            double verticalVelocity =
                    velocity * Math.sin(theta)
                            - GRAVITY * time;

            // Require the ball to be descending.
            if (verticalVelocity > -MIN_DESCENT_SPEED) {
                continue;
            }

            double wheelRPM =
                    velocity * 60.0
                            / (Math.PI
                            * FLYWHEEL_DIAMETER_M
                            * BALL_SPEED_FACTOR);

            double requiredMotorRPM =
                    wheelRPM / WHEEL_REVS_PER_MOTOR_REV;

            if (!Double.isFinite(requiredMotorRPM)
                    || requiredMotorRPM <= 0
                    || requiredMotorRPM > MAX_MOTOR_RPM) {
                continue;
            }

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

    private double clamp(
            double value,
            double minimum,
            double maximum) {

        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }

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

                    alignTimer.reset();

                    limelight.pipelineSwitch(TAG_PIPELINE);
                    follower.manual(0, 0, 0);
                })

                .setExecute(() -> {

                    if (alignTimer.seconds()
                            < PIPELINE_SETTLE) {

                        follower.manual(0, 0, 0);
                        return;
                    }

                    if (!updateTargetGeometry()) {

                        status = "Waiting for four HIVE tags";
                        lockStarted = false;

                        follower.manual(0, 0, 0);
                        return;
                    }

                    double error = yawErrorDeg;

                    if (Math.abs(error)
                            <= ALIGN_TOLERANCE) {

                        follower.manual(0, 0, 0);

                        if (!lockStarted) {
                            lockStarted = true;
                            lockTimer.reset();
                        }

                        if (lockTimer.seconds() >= LOCK_TIME) {

                            aligned = true;
                            calculateShot();
                        }

                        return;
                    }

                    lockStarted = false;
                    status = "Aligning to HIVE";

                    double turn =
                            TURN_SIGN * ALIGN_KP * error;

                    if (Math.abs(turn) < MIN_TURN) {
                        turn = Math.copySign(
                                MIN_TURN,
                                turn
                        );
                    }

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
