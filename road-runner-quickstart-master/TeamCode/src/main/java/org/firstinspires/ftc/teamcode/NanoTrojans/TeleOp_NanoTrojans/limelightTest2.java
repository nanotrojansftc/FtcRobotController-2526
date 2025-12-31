package org.firstinspires.ftc.teamcode.NanoTrojans.TeleOp_NanoTrojans;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Autonomous(name = "Limelight AprilTag Test")
public class limelightTest2 extends OpMode {

    private Limelight3A limelight3A;
    private IMU imu;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

        // IMPORTANT: Pipeline must be an APRILTAG pipeline
        limelight3A.pipelineSwitch(0);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        limelight3A.start();
    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {

        // Update robot yaw for accurate botpose
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight3A.getLatestResult();

        if (llResult != null && llResult.isValid()) {

            // ----- AprilTag (Fiducial) Detection -----
            List<LLResultTypes.FiducialResult> fiducials =
                    llResult.getFiducialResults();

            telemetry.addData("AprilTags Detected", fiducials.size());

            for (LLResultTypes.FiducialResult tag : fiducials) {
                telemetry.addData("AprilTag ID", tag.getFiducialId());
                //21 GPP    22 PGP   23 PPG
            }

            // ----- Bot Pose from AprilTags -----
            Pose3D botPose = llResult.getBotpose_MT2();
            if (botPose != null) {
                telemetry.addData("BotPose", botPose.toString());
                telemetry.addData("Bot Yaw", botPose.getOrientation().getYaw());
            }

            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.update();
        }
        telemetry.addData("Test", 2);
        telemetry.update();
    }
}
