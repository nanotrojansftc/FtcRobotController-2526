package org.firstinspires.ftc.teamcode.NanoTrojans.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.NanoTrojans.Subsystems.IntakeSubsystem;

@Autonomous(name = "Test: Smart Intake", group = "Diagnostics")
public class TestIntakeSystem extends LinearOpMode {
    @Override
    public void runOpMode() {
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            intake.runIntakeAutomation(false);

            telemetry.addLine("--- SENSOR DATA ---");
            telemetry.addData("Intake Raw", intake.getIntakeRawSum());
            telemetry.addData("Intake Detected", intake.artifactDetectedAtIntake());
            telemetry.addData("Left Bench Full", intake.isLeftBenchFull());
            telemetry.addData("Right Bench Full", intake.isRightBenchFull());

            telemetry.addLine("--- STATS ---");
            // This fulfills your "Intook One" request
            telemetry.addData("Status", "Intook %d artifacts", intake.getArtifactsIntakenCount());
            telemetry.addData("Spindexer Slot", intake.getArtifactsIntakenCount());

            telemetry.update();
        }
    }
}