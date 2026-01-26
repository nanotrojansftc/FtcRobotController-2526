package org.firstinspires.ftc.teamcode.NanoTrojans.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PDF Tuner in PIDF Tuner", group = "Tuning")
public class PDFTunerInPIDFTunerForSingleMotors extends LinearOpMode {
    private DcMotorEx spindexerMotor;

    // --- STARTING CONSTANTS ---
    // Note: kP is very sensitive. kF is for friction. kD is for braking.
    double kP = 0.002;
    double kF = 0.05;
    double kD = 0.000;

    // --- VARIABLES ---
    int targetPosition = 0;
    int ticksPerCompartment = 250; // ~1/3rd rotation for 26.9:1 motor
    double lastError = 0;

    // 0=P, 1=F, 2=D
    int tuningMode = 1; // Start with F (Friction) as it's the first step!

    @Override
    public void runOpMode() throws InterruptedException {

        // HARDWARE SETUP
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // INITIAL TELEMETRY
        telemetry.addLine("--- SPINDEXER TUNER ---");
        telemetry.addLine("1. Mount robot safely (wheels off ground if needed)");
        telemetry.addLine("2. Press START to begin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 1. CONTROLS ---

            // Switch Mode (Y)
            if (gamepad1.y) {
                tuningMode++;
                if (tuningMode > 2) tuningMode = 0;
                sleep(250);
            }

            // Adjust Values (D-Pad)
            if (gamepad1.dpad_up || gamepad1.dpad_down) {
                double direction = gamepad1.dpad_up ? 1 : -1;

                if (tuningMode == 0) kP += (0.0001 * direction);      // Small steps for P
                else if (tuningMode == 1) kF += (0.005 * direction);  // Medium steps for F
                else if (tuningMode == 2) kD += (0.00001 * direction); // Tiny steps for D

                // Prevent negatives
                kP = Math.max(0, kP);
                kF = Math.max(0, kF);
                kD = Math.max(0, kD);
                sleep(100);
            }

            // Test Movement (A/B)
            if (gamepad1.a) { targetPosition += ticksPerCompartment; sleep(200); }
            if (gamepad1.b) { targetPosition -= ticksPerCompartment; sleep(200); }

            // --- 2. MATH ---
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

            // --- 3. INSTRUCTIONAL TELEMETRY ---

            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("[Y]: Switch Variable (P -> F -> D)");
            telemetry.addLine("[D-Pad]: Adjust Value");
            telemetry.addLine("[A] / [B]: Rotate Left / Right");
            telemetry.addLine(" "); // Spacer

            telemetry.addLine("=== CURRENT MODE ===");
            if (tuningMode == 1) {
                telemetry.addData("EDITING", "kF (Feedforward/Friction)");
                telemetry.addData("Value", "%.4f", kF);
                telemetry.addLine("TIP: Increase until motor *barely* moves/hums.");
                telemetry.addLine("GOAL: Overcome friction, but don't spin yet.");
            }
            else if (tuningMode == 0) {
                telemetry.addData("EDITING", "kP (Proportional/Strength)");
                telemetry.addData("Value", "%.5f", kP);
                telemetry.addLine("TIP: Increase until it snaps to target.");
                telemetry.addLine("WARN: If it shakes/oscillates -> DECREASE.");
            }
            else if (tuningMode == 2) {
                telemetry.addData("EDITING", "kD (Derivative/Brake)");
                telemetry.addData("Value", "%.6f", kD);
                telemetry.addLine("TIP: Use ONLY if it overshoots/bounces.");
                telemetry.addLine("WARN: Keep very low. Too high = Gritty noise.");
            }

            telemetry.addLine(" "); // Spacer
            telemetry.addLine("=== DATA ===");
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Actual", currentPos);
            telemetry.addData("Error", error);
            telemetry.addData("Motor Pwr", "%.2f", power);

            telemetry.update();
        }
    }
}