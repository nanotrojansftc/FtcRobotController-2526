package org.firstinspires.ftc.teamcode.NanoTrojans.TeleOp_NanoTrojans;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Autonomous(name = "Limelight3")
public class limelightTest3 extends OpMode {

    private Limelight3A limelight3A;
    private IMU imu;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

        // IMPORTANT: Pipeline must be an APRILTAG pipeline
        limelight3A.pipelineSwitch(0);
        limelight3A.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)

        telemetry.setMsTransmissionInterval(11);


        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight3A.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

    @Override
    public void loop() {

        LLStatus status = limelight3A.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        // Update robot yaw for accurate botpose
        LLResult result = limelight3A.getLatestResult();
        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducials =
                    result.getFiducialResults();

            telemetry.addData("AprilTags Detected", fiducials.size());

            for (LLResultTypes.FiducialResult tag : fiducials) {
                telemetry.addData("AprilTag ID", tag.getFiducialId());
                //21 GPP    22 PGP   23 PPG
            }

            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.update();
    }
}
