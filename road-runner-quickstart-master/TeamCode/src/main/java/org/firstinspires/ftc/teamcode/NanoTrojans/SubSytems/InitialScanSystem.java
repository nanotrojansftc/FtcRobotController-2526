package org.firstinspires.ftc.teamcode.NanoTrojans.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

public class InitialScanSystem {
    private Limelight3A limelight;
    private int detectedObeliskId = -1;

    public InitialScanSystem(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");

    }

    // Move these out of the constructor
    public void initLimelight() {
        if (limelight != null) {
            limelight.setPollRateHz(100); //
            limelight.pipelineSwitch(0); //
            limelight.start(); //
        }
    }

    public void scanObelisk() {
        // Use getLatestResult() to pull the freshest frame.
        LLResult result = limelight.getLatestResult();

        // CHECK 1: Ensure result is not null and is valid.
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            // CHECK 2: Loop through all detected tags.
            for (LLResultTypes.FiducialResult f : fiducials) {
                int id = f.getFiducialId();
                // Filter for Obelisk Tags only (21, 22, 23).
                if (id >= 21 && id <= 23) {
                    detectedObeliskId = id;
                }
            }
        }
    }

    public int getObeliskId() {
        return detectedObeliskId;
    }

    public boolean isReady() {
        return detectedObeliskId != -1;
    }
}