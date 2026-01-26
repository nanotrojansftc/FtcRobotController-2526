package org.firstinspires.ftc.teamcode.NanoTrojans.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hood Servo Calibration", group = "Tools")
public class HoodServoCalibration extends LinearOpMode {
    private Servo hoodServo;
    private double currentPosition = 0.5; // Start halfway

    @Override
    public void runOpMode() {
        // Hardware Mapping
        hoodServo = hardwareMap.get(Servo.class, "hood");

        // UNCOMMENT if your servo runs backwards relative to the angle
        // hoodServo.setDirection(Servo.Direction.REVERSE);

        hoodServo.setPosition(currentPosition);

        telemetry.addLine("Initialized.");
        telemetry.addLine("Use DPAD UP/DOWN for fine adjust (+/- 0.001)");
        telemetry.addLine("Use BUMPERS for coarse adjust (+/- 0.05)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Coarse Adjust
            if (gamepad1.right_bumper) {
                currentPosition += 0.002; // Fast speed
            } else if (gamepad1.left_bumper) {
                currentPosition -= 0.002;
            }

            // Fine Adjust (Single Press logic can be tricky in loop,
            // so we just use a slow increment for holding)
            if (gamepad1.dpad_up) {
                currentPosition += 0.0005; // Very slow speed
            } else if (gamepad1.dpad_down) {
                currentPosition -= 0.0005;
            }

            // Safety Clamp (0.0 to 1.0)
            currentPosition = Math.max(0.0, Math.min(1.0, currentPosition));

            // Set the Servo
            hoodServo.setPosition(currentPosition);

            // Feedback
            telemetry.addData("--- SERVO CALIBRATION ---", "");
            telemetry.addData("Current Position", "%.4f", currentPosition);
            telemetry.addLine("\nINSTRUCTIONS:");
            telemetry.addLine("1. Move hood to a Low Angle.");
            telemetry.addLine("2. Measure physical angle (e.g., 10 deg).");
            telemetry.addLine("3. Write down Position & Angle for Constant Set 1.");
            telemetry.addLine("\n4. Move hood to a High Angle.");
            telemetry.addLine("5. Measure physical angle (e.g., 60 deg).");
            telemetry.addLine("6. Write down Position & Angle for Constant Set 2.");

            telemetry.update();
        }
    }



}
