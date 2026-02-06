package org.firstinspires.ftc.teamcode.NanoTrojans.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.NanoTrojans.Subsystems.InitialScanSystem;

@Autonomous(name = "Test: Scan System", group = "Diagnostics")
public class TestScanSystem extends LinearOpMode {
    @Override
    public void runOpMode() {
        InitialScanSystem scanner = new InitialScanSystem(hardwareMap);

        // Call the init here so it's safer
        scanner.initLimelight();

        // IMPORTANT: Ensure the Limelight is actually commanded to start
        // and set to the correct pipeline at the very beginning.
        // This is often missing in failed scans.

        telemetry.addData("Status", "Limelight Starting...");
        telemetry.update();

        // Give the hardware a second to heartbeat
        sleep(1000);

        waitForStart(); // Ensure we wait for the user to hit Play

        while (opModeIsActive()) {
            scanner.scanObelisk();
            // ADD THIS LINE to see if the camera is even "Valid" to the code
            // If this stays -1, check your hardware config name "limelight"
            int id = scanner.getObeliskId();

            telemetry.addData("Target ID Detected", id);
            telemetry.addData("Ready?", scanner.isReady());
            telemetry.update();
        }
    }
}