package org.firstinspires.ftc.teamcode.NanoTrojans.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top;

@TeleOp(name = "Spindexer Tuner", group = "Tuning")
public class SpindexerTuner extends LinearOpMode {

    // Declare the resources (motors/servos)
    private resources_top resources;

    // Variables for tuning
    // We start at 300ms since speed servos are faster than the original 474ms
    long spinTime = 410;
    int carousel = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware using your existing resources class
        resources = new resources_top(hardwareMap);

        // -- TELEMETRY INSTRUCTIONS --
        telemetry.addLine("--- Spindexer Tuner ---");
        telemetry.addLine("Controls:");
        telemetry.addLine("[D-Pad Up/Down]: Adjust Time (+/- 10ms)");
        telemetry.addLine("[Button A]: Test Rotation");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- ADJUST TIME ---
            // Dpad Up increases time
            if (gamepad1.dpad_up) {
                spinTime += 10;
                sleep(200); // Small delay to prevent zooming too fast
            }
            // Dpad Down decreases time
            else if (gamepad1.dpad_down) {
                spinTime -= 10;
                if (spinTime < 0) spinTime = 0; // Safety clamp
                sleep(200);
            }

            // --- TEST ROTATION ---
            if (gamepad1.a) {
                performRotation();
                // Wait for button release so it doesn't fire twice
                while (opModeIsActive() && gamepad1.a) { idle(); }
            }

            // --- TELEMETRY ---
            telemetry.addData(">> Current Spin Time (ms)", spinTime);
            telemetry.addData("Carousel State", carousel);
            telemetry.addData("Next Servo", getServoName(carousel));
            telemetry.addLine("\nPress 'A' to test rotation.");
            telemetry.update();
        }
    }

    // Logic copied from TeleOpMainMC but using dynamic 'spinTime'
    private void performRotation() {
        if (carousel == 2) {
            carousel = 0;
            resources.lspin.setPower(-1);
            sleep(spinTime);
            resources.lspin.setPower(0);
        } else if (carousel == 1) {
            carousel += 1;
            resources.rspin.setPower(-1);
            sleep(spinTime);
            resources.rspin.setPower(0);
        } else if (carousel == 0) {
            carousel += 1;
            resources.fspin.setPower(-1);
            sleep(spinTime);
            resources.fspin.setPower(0);
        }
    }

    // Helper to show which servo is about to move
    private String getServoName(int state) {
        if (state == 0) return "Front (fspin)";
        if (state == 1) return "Right (rspin)";
        return "Left (lspin)";
    }
}