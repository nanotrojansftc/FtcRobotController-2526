package org.firstinspires.ftc.teamcode.NanoTrojans.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;
import java.util.ArrayList;
import java.util.List;

public class InitialScanSystem {
    private Limelight3A limelight;
    private colorsensors bench;

    // Result Storage
    private int detectedObeliskId = -1;
    private List<String> startingInventory = new ArrayList<>();

    public InitialScanSystem(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        bench = new colorsensors();
        bench.init(hwMap);
    }

    /**
     * Scans for AprilTags 21, 22, or 23 on the Obelisk.
     * This should be called repeatedly in the Init Loop.
     */
    public void scanObelisk() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tags) {
                int id = tag.getFiducialId();
                // Filter specifically for the competition Obelisk tags
                if (id >= 21 && id <= 23) {
                    detectedObeliskId = id;
                }
            }
        }
    }

    /**
     * Scans the back two chambers to see what we are starting with.
     */
    public void scanStartingInventory() {
        startingInventory.clear();

        colorsensors.DetectedColor left = bench.detectByHue(bench.left, null);
        colorsensors.DetectedColor right = bench.detectByHue(bench.right, null);

        startingInventory.add(left.toString());
        startingInventory.add(right.toString());
    }

    public int getObeliskId() {
        return detectedObeliskId;
    }

    public List<String> getInventory() {
        return startingInventory;
    }

    public boolean isReady() {
        return detectedObeliskId != -1;
    }
}