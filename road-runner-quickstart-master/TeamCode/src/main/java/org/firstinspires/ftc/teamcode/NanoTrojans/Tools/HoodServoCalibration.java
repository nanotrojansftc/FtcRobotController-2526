package org.firstinspires.ftc.teamcode.NanoTrojans.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;

@TeleOp(name = "Hood Calibration", group = "Tools")
public class HoodServoCalibration extends LinearOpMode {

    // --- HARDWARE ---
    private Servo hoodServo;
    private DcMotorEx LflywheelMotor, RflywheelMotor;
    private Limelight3A limelight;

    // --- STATE ---
    private double currentServoPos = 0.5;
    private boolean shooterEnabled = false;

    // --- MEASUREMENTS (Must match SmartShooter.java) ---
    final double CAMERA_HEIGHT_INCHES = 16.25;
    final double TAG_HEIGHT_INCHES    = 29.5;
    final double MOUNT_ANGLE_DEGREES  = 10.6;
    final double SHOOTER_HEIGHT       = 17.7;
    final double BASKET_HEIGHT        = 43.0;
    final double ENTRY_ANGLE          = -45.0; // Fixed entry angle from SmartShooter
    final double G                    = 386.1;

    // --- MOTOR CONSTANTS ---
    final double FLYWHEEL_TICKS_PER_REV = 28.0;
    final double FLYWHEEL_RADIUS = 1.89;
    final double SPEED_SCALAR = 3;

    @Override
    public void runOpMode() {
        // 1. HARDWARE INIT
        hoodServo = hardwareMap.get(Servo.class, "hood");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // 2. MOTOR INIT (EXACT PIDF COPY FROM SMARTSHOOTER)
        LflywheelMotor = hardwareMap.get(DcMotorEx.class, "lgun");
        LflywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 0, 1.5, 16.45));

        RflywheelMotor = hardwareMap.get(DcMotorEx.class, "rgun");
        RflywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 0, 1.5, 16.45));

        hoodServo.setPosition(currentServoPos);

        telemetry.setMsTransmissionInterval(50); // Fast updates
        telemetry.addLine("Initialized.");
        telemetry.addLine("Press 'A' to toggle Flywheels ON/OFF");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            // Default values
            double currentDist = 0;
            double requiredPhysicsAngle = 0;
            double targetRPM = 0;
            double leftTargetTicks = 0;
            double rightTargetTicks = 0;
            int tagID = -1;
            boolean validTarget = false;

            // --- LOGIC LOOP ---
            if (result != null && result.isValid()) {
                // Get Tag ID
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (!tags.isEmpty()) { tagID = tags.get(0).getFiducialId(); }

                // 1. Calculate Distance
                double ty = result.getTy();
                double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEGREES + ty);
                currentDist = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

                // 2. Calculate Physics (Angle & Velocity)
                // This uses the EXACT SAME logic as SmartShooter
                ShotData shot = calculateShot(currentDist, BASKET_HEIGHT - SHOOTER_HEIGHT, ENTRY_ANGLE);

                if (shot.isPossible) {
                    validTarget = true;
                    requiredPhysicsAngle = shot.launchAngleDegrees;

                    // 3. Calculate RPM
                    targetRPM = (shot.launchVelocityInchesPerSec * 60) / (2 * Math.PI * FLYWHEEL_RADIUS);
                    targetRPM *= SPEED_SCALAR; // Apply the 3x scalar

                    double baseTicks = (targetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;
                    leftTargetTicks = baseTicks;
                    rightTargetTicks = baseTicks;

                    // 4. Apply Long Range Boost (From SmartShooter)
                    if (currentDist > 120.0) {
                        leftTargetTicks *= 1.80;
                    }
                }
            }

            // --- SERVO ADJUSTMENT (Manual) ---
            if (gamepad1.right_bumper) currentServoPos += 0.002;
            else if (gamepad1.left_bumper) currentServoPos -= 0.002;
            if (gamepad1.dpad_up) currentServoPos += 0.0005;
            else if (gamepad1.dpad_down) currentServoPos -= 0.0005;

            currentServoPos = Math.max(0.0, Math.min(1.0, currentServoPos));
            hoodServo.setPosition(currentServoPos);

            // --- MOTOR CONTROL ---
            // Toggle 'A' with debounce (wait until release logic simplified here)
            if (gamepad1.a && !shooterEnabled) { shooterEnabled = true; sleep(250); }
            else if (gamepad1.a && shooterEnabled) { shooterEnabled = false; sleep(250); }

            if (shooterEnabled && validTarget) {
                LflywheelMotor.setVelocity(leftTargetTicks);
                RflywheelMotor.setVelocity(rightTargetTicks);
            } else {
                LflywheelMotor.setVelocity(0);
                RflywheelMotor.setVelocity(0);
            }

            // --- TELEMETRY ---
            double leftActualRPM = (LflywheelMotor.getVelocity() * 60) / FLYWHEEL_TICKS_PER_REV;
            double rightActualRPM = (RflywheelMotor.getVelocity() * 60) / FLYWHEEL_TICKS_PER_REV;

            telemetry.addData("--- SYSTEM STATUS ---", shooterEnabled ? "RUNNING" : "IDLE");

            if (validTarget) {
                telemetry.addData("Tag ID", tagID);
                telemetry.addData("Distance", "%.1f inches", currentDist);
                telemetry.addData("---------------------", "");
                telemetry.addData(">> PHYSICS ANGLE", "%.2f deg", requiredPhysicsAngle);
                telemetry.addData(">> SERVO POS", "%.4f", currentServoPos);
                telemetry.addData("---------------------", "");
                telemetry.addData("Target RPM", "%.0f", targetRPM);
                telemetry.addData("L RPM (Actual)", "%.0f", leftActualRPM);
                telemetry.addData("R RPM (Actual)", "%.0f", rightActualRPM);
            } else {
                telemetry.addLine("SEARCHING FOR TARGET...");
            }
            telemetry.update();
        }
        limelight.stop();
    }

    // Exact Logic from SmartShooter.java
    private ShotData calculateShot(double x, double y, double theta) {
        ShotData data = new ShotData();
        double thetaRad = Math.toRadians(theta);

        double term1 = (2 * y) / x;
        double term2 = Math.tan(thetaRad);
        double alphaRad = Math.atan(term1 - term2);
        data.launchAngleDegrees = Math.toDegrees(alphaRad);

        double cosAlpha = Math.cos(alphaRad);
        double tanAlpha = Math.tan(alphaRad);
        double numerator = G * Math.pow(x, 2);
        double denominator = 2 * Math.pow(cosAlpha, 2) * ((x * tanAlpha) - y);

        if (denominator <= 0) {
            data.isPossible = false;
        } else {
            data.isPossible = true;
            data.launchVelocityInchesPerSec = Math.sqrt(numerator / denominator);
        }
        return data;
    }

    private class ShotData {
        public double launchAngleDegrees;
        public double launchVelocityInchesPerSec;
        public boolean isPossible;
    }
}