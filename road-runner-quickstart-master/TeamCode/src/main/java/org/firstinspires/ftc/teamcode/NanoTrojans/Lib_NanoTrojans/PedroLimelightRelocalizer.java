package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class PedroLimelightRelocalizer {

    private Follower follower;
    private Limelight3A limelight;

    public PedroLimelightRelocalizer(Follower follower, HardwareMap hardwareMap) {
        this.follower = follower;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    /**
     * Updates the robot's absolute position using the tag.
     * WARNING: We are NOT calling this in the loop anymore to prevent jitters.
     */
    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botPose3D = result.getBotpose();
            if (botPose3D != null) {
                // Convert Meters to Inches
                double x_inches = botPose3D.getPosition().x * 39.3701;
                double y_inches = botPose3D.getPosition().y * 39.3701;
                double currentHeading = follower.getPose().getHeading();

                // Reset Pedro's position
                Pose correctedPose = new Pose(x_inches, y_inches, currentHeading);
                follower.setPose(correctedPose);
            }
        }
    }

    /**
     * Gets the horizontal angle to the tag (for aiming).
     */
    public double getOffsetFromTag(int tagID) {
        // 1. Get the latest result
        LLResult result = limelight.getLatestResult();

        // 2. Return tx if valid
        if (result != null && result.isValid()) {
            return result.getTx(); // <--- FIXED: called on result, not limelight
        }
        return -999;
    }

    public void switchPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }
}