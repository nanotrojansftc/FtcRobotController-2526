package org.firstinspires.ftc.teamcode.NanoTrojans.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "PDF Tuner Fixed (Slot Logic)", group = "Tuning")
public class PDFTunerInPIDFTunerForSingleMotors extends LinearOpMode {
    private DcMotorEx spindexerMotor;

    // --- PIDF CONSTANTS ---
    double kP = 0.002;
    double kF = 0.05;
    double kD = 0.000;

    // --- VARIABLES ---
    int targetPosition = 0;

    // STARTING GUESS for the decimal ticks (Adjustable in Mode 3)
    double ticksPerCompartment = 250.6;

    // NEW: We track the "Slot" we are in (0, 1, 2, 3...)
    // This prevents the rounding error from adding up!
    int currentSlot = 0;
    double lastError = 0;

    // TUNING MODES:
    // 0 = P (Strength)
    // 1 = F (Friction)
    // 2 = D (Braking)
    // 3 = TICKS (The Magic Number)
    int tuningMode = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        // HARDWARE SETUP
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexer"); // Check if your config is "hood" or "spindexer"
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // INITIAL TELEMETRY
        telemetry.addLine("--- SPINDEXER TUNER (FIXED) ---");
        telemetry.addLine("1. Align Spindexer to FRONT (Position 0)");
        telemetry.addLine("2. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 1. CONTROLS ---

            // Switch Mode (Y) -> Cycles 0, 1, 2, 3
            if (gamepad1.y) {
                tuningMode++;
                if (tuningMode > 3) tuningMode = 0;
                sleep(250);
            }

            // Adjust Values (D-Pad Up/Down)
            if (gamepad1.dpad_up || gamepad1.dpad_down) {
                double direction = gamepad1.dpad_up ? 1 : -1;

                if (tuningMode == 0) kP += (0.0001 * direction);       // Tune P
                else if (tuningMode == 1) kF += (0.005 * direction);   // Tune F
                else if (tuningMode == 2) kD += (0.00001 * direction); // Tune D
                else if (tuningMode == 3) ticksPerCompartment += (0.1 * direction); // Tune Magic Number

                // Prevent negatives
                kP = Math.max(0, kP);
                kF = Math.max(0, kF);
                kD = Math.max(0, kD);
                sleep(100);
            }

            // ROTATE SLOTS (A / B)
            // This now uses the "Bank Account" logic to fix the 250 vs 251 issue
            if (gamepad1.a) {
                currentSlot++;
                updateTarget();
                sleep(200);
            }
            if (gamepad1.b) {
                currentSlot--;
                updateTarget();
                sleep(200);
            }

            // --- 2. PIDF MATH ---
            int currentPos = spindexerMotor.getCurrentPosition();
            double error = targetPosition - currentPos;

            double pTerm = error * kP;
            double fTerm = 0;
            if (Math.abs(error) > 5) fTerm = Math.signum(error) * kF;
            double dTerm = (error - lastError) * kD;
            lastError = error;

            double power = pTerm + fTerm + dTerm;
            power = Math.max(-1.0, Math.min(1.0, power));
            spindexerMotor.setPower(power);

            // --- 3. TELEMETRY ---
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("[Y]: Change Mode  |  [D-Pad]: Adjust Value");
            telemetry.addLine("[A/B]: Rotate Slot (+/-)");
            telemetry.addLine(" ");

            telemetry.addLine("=== TUNING MODE ===");
            if (tuningMode == 1) {
                telemetry.addData(">> EDITING", "F (FRICTION)");
                telemetry.addData("Value", "%.4f", kF);
            } else if (tuningMode == 0) {
                telemetry.addData(">> EDITING", "P (STRENGTH)");
                telemetry.addData("Value", "%.5f", kP);
            } else if (tuningMode == 2) {
                telemetry.addData(">> EDITING", "D (BRAKE)");
                telemetry.addData("Value", "%.6f", kD);
            } else if (tuningMode == 3) {
                telemetry.addData(">> EDITING", "TICKS (MAGIC NUMBER)");
                telemetry.addData("Value", "%.1f", ticksPerCompartment);
                telemetry.addLine("Adjust this until 10 rotations land PERFECTLY.");
            }

            telemetry.addLine(" ");
            telemetry.addData("Slot #", currentSlot);
            telemetry.addData("Target (Int)", targetPosition);
            telemetry.addData("Actual", currentPos);
            telemetry.addData("Error", error);
            telemetry.update();
        }
    }

    // Helper to calculate the perfect target with rounding
    private void updateTarget() {
        double preciseTarget = currentSlot * ticksPerCompartment;
        targetPosition = (int) Math.round(preciseTarget);
    }
}