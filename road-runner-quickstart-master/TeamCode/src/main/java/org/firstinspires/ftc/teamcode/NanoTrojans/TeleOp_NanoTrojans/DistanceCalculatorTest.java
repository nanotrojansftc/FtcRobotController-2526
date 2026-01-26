package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;

@TeleOp(name = "Limelight Distance Test", group = "Sensor")
public class DistanceCalculatorTest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // -------------------------------------------------------------------------
        // 1. HARDWARE MAPPING
        // -------------------------------------------------------------------------
        // Make sure the device is named "limelight" in your Driver Station Config
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // -------------------------------------------------------------------------
        // 2. CONFIGURATION
        // -------------------------------------------------------------------------
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0); // Setup Pipeline 0 on your PC for AprilTags

        /*
         * Starts polling for data.
         * If you haven't configured the Limelight via computer yet:
         * 1. Connect via USB/Ethernet
         * 2. Go to http://172.29.0.1:5801 (or correct IP)
         * 3. Set "Pipeline Type" to "Fiducial Markers" (AprilTags)
         */
        limelight.start();

        telemetry.addLine("Ready. Press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                // --- METHOD 1: TRIGONOMETRY (Using your measurements) ---
                double targetOffsetAngle_Vertical = result.getTy();

                // PHYSICAL CONSTANTS (Update these if you move the camera!)
                double cameraHeight = 17.5;  // Inches from floor to lens
                double targetHeight = 30.0;  // Inches from floor to center of Tag #24
                double cameraAngle  = 9.6;   // Degrees camera is tilted UP from horizontal

                // THE MATH
                double angleToGoalDegrees = cameraAngle + targetOffsetAngle_Vertical;
                double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

                // (30 - 17.5) = 12.5 inch difference
                double distTrig = (targetHeight - cameraHeight) / Math.tan(angleToGoalRadians);


                // --- METHOD 2: BUILT-IN 3D (Automatic & More Accurate) ---
                double dist3D = 0;
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (!tags.isEmpty()) {
                    // "z" is the forward distance in meters. Multiply by 39.37 to get inches.
                    dist3D = tags.get(0).getCameraPoseTargetSpace().getPosition().z * 39.37;
                }

                // --- TELEMETRY ---
                telemetry.addData("Target Found", "YES");
                telemetry.addData("Tag ID", tags.isEmpty() ? "None" : tags.get(0).getFiducialId());
                telemetry.addLine("--------------------------------");
                telemetry.addData("Calculated Distance (Trig)", "%.2f inches", distTrig);
                telemetry.addData("Actual Distance (3D)",       "%.2f inches", dist3D);
                telemetry.addLine("--------------------------------");
                telemetry.addData("ty (Vertical Offset)", "%.2f", targetOffsetAngle_Vertical);

            } else {
                telemetry.addData("Target Found", "NO");
            }
            telemetry.update();
        }

        limelight.stop();
    }
}