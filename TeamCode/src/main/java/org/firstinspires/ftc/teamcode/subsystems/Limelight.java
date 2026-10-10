package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Subsystem wrapping the Limelight 3A vision sensor and REV Control Hub IMU.
 * Provides target tracking data (Tx, Ty, Ta) and MegaTag2 3D field localization poses.
 */
public class Limelight {

    // ---------------------- Hardware Devices ----------------------
    private Limelight3A limelight;
    private IMU imu;

    // ---------------------- Tracking State Variables ----------------------
    private Pose3D botPose;
    private boolean targetVisible = false;
    private double horizontalDelta = 0; // Tx: horizontal offset angle to target
    private double verticalDelta = 0;   // Ty: vertical offset angle to target
    private double targetArea = 0;      // Ta: target area (% of camera image)

    public static final String TAG = "Limelight";

    /**
     * Initializes Limelight 3A camera and REV Hub IMU orientation.
     * Configures default pipeline index 0.
     */
    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        Log.d(TAG, "Limelight found " + limelight);
        try {
            imu = hwMap.get(IMU.class, "imu");
            Log.d(TAG, "IMU found " + imu);
            RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            );
            imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        } catch (Exception e) {
            imu = null; // IMU optional fallback
        }
        limelight.pipelineSwitch(0);
    }

    /**
     * Starts camera polling and processing on the Limelight.
     */
    public void start() {
        limelight.start();
    }

    /**
     * Stops camera polling and processing.
     */
    public void stop() {
        limelight.stop();
    }

    /**
     * Updates IMU orientation feed and processes latest Limelight vision result.
     */
    public void update() {
        handlePosition(getLimelightResult());
    }

    /**
     * Updates robot yaw orientation on Limelight for MegaTag2 localization and returns latest result.
     */
    public LLResult getLimelightResult() {
        if (imu != null) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
        }
        return limelight.getLatestResult();
    }

    /**
     * Parses the Limelight 3D pose, horizontal/vertical target deltas, and logs diagnostic output.
     */
    private void handlePosition(LLResult llResult) {
        if (llResult != null && llResult.isValid()) {
            if (imu != null) {
                botPose = llResult.getBotpose_MT2(); // MegaTag 2 IMU-assisted 3D pose
            } else {
                botPose = llResult.getBotpose();     // Standard 3D botpose
            }
            targetVisible = true;
            double tx = llResult.getTx();
            double ty = llResult.getTy();
            double ta = llResult.getTa();

            if (tx != horizontalDelta || ty != verticalDelta || ta != targetArea) {
                this.horizontalDelta = tx;
                this.verticalDelta = ty;
                this.targetArea = ta;
                if (botPose != null) {
                    Log.d(TAG, "Tx:" + horizontalDelta + " Ty:" + verticalDelta + " Ta:" + targetArea +
                            " X:" + botPose.getPosition().x + " Y:" + botPose.getPosition().y + " Z:" + botPose.getPosition().z +
                            " Yaw:" + botPose.getOrientation().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES) +
                            " Pitch:" + botPose.getOrientation().getPitch(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES) +
                            " Roll:" + botPose.getOrientation().getRoll(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES) +
                            " IsValid:" + llResult.isValid());
                } else {
                    Log.d(TAG, "Tx:" + horizontalDelta + " Ty:" + verticalDelta + " Ta: " + targetArea + " IsValid: " + llResult.isValid());
                }
            }
        } else {
            targetVisible = false;
            horizontalDelta = 0;
            verticalDelta = 0;
            targetArea = 0;
        }
    }

    // --------------------- Getters ---------------------
    public double getHorizontalDelta() {
        return horizontalDelta;
    }

    public double getVerticalDelta() {
        return verticalDelta;
    }

    public double getTargetArea() {
        return targetArea;
    }

    public Pose3D getBotPose() {
        return botPose;
    }

    public double get3DXDistance() {
        if (botPose != null) {
            return botPose.getPosition().x;
        }
        return 0;
    }

    public double get3DYDistance() {
        if (botPose != null) {
            return botPose.getPosition().y;
        }
        return 0;
    }

    public boolean isTargetVisible() {
        return targetVisible;
    }
}
