package org.firstinspires.ftc.teamcode.NanoTrojans.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Smart Shooter - Servo Mode", group = "Competition")
public class SmartShooterWithServo extends LinearOpMode {

    // --- HARDWARE ---
    private Limelight3A limelight;
    private DcMotorEx LflywheelMotor;
    private DcMotorEx RflywheelMotor;
    private Servo hoodServo;

    // --- MEASUREMENTS (Same as SmartShooter) ---
    final double CAMERA_HEIGHT_INCHES = 16.25;
    final double TAG_HEIGHT_INCHES    = 29.5;
    final double MOUNT_ANGLE_DEGREES  = 10.6;
    final double SHOOTER_HEIGHT       = 17.7;
    final double BASKET_HEIGHT        = 43.0;
    final double ENTRY_ANGLE          = -45.0; // Keeps original -45 logic

    // --- SERVO LOGIC (Your Request) ---
    final double SERVO_LOW_POS  = 0.5;   // Distance < 90
    final double SERVO_HIGH_POS = 0.58;  // Distance > 90
    final double SERVO_THRESHOLD_DIST = 90.0;

    // --- MOTOR CONSTANTS (Same as SmartShooter) ---
    final double FLYWHEEL_TICKS_PER_REV = 28.0;
    final double FLYWHEEL_RADIUS = 1.89;
    final double SPEED_SCALAR = 3;

    @Override
    public void runOpMode() {
        // 1. INIT HARDWARE
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // 2. INIT FLYWHEELS (Exact PIDF from SmartShooter)
        LflywheelMotor = hardwareMap.get(DcMotorEx.class, "lgun");
        LflywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 0, 1.5, 16.45));

        RflywheelMotor = hardwareMap.get(DcMotorEx.class, "rgun");
        RflywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 0, 1.5, 16.45));

        // 3. INIT HOOD SERVO
        hoodServo = hardwareMap.get(Servo.class, "hood");

        // Logic: "Reset back to .5 on the start"
        hoodServo.setPosition(SERVO_LOW_POS);

        telemetry.addData("Status", "Initialized. Hood Reset to 0.5");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            double currentDist = 0;
            double targetRPM = 0;
            double leftVelocity = 0;
            double rightVelocity = 0;
            String servoStatus = "LOW (0.5)";

            if (result != null && result.isValid()) {
                // --- STEP 1: DISTANCE (Same logic) ---
                double ty = result.getTy();
                double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEGREES + ty);
                currentDist = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

                // --- STEP 2: PHYSICS (Same logic) ---
                double heightY = BASKET_HEIGHT - SHOOTER_HEIGHT;
                ShotData shot = calculateShot(currentDist, heightY, ENTRY_ANGLE);

                if (shot.isPossible) {
                    // --- STEP 3: SERVO LOGIC (New Request) ---
                    // "If under 90 inches then lowest (0.5). If over 90 inches, 0.58."
                    if (currentDist < SERVO_THRESHOLD_DIST) {
                        hoodServo.setPosition(SERVO_LOW_POS);
                        servoStatus = "LOW (0.5)";
                    } else {
                        hoodServo.setPosition(SERVO_HIGH_POS);
                        servoStatus = "HIGH (0.58)";
                    }

                    // --- STEP 4: FLYWHEEL CONTROL (Same logic) ---
                    targetRPM = (shot.launchVelocityInchesPerSec * 60) / (2 * Math.PI * FLYWHEEL_RADIUS);
                    targetRPM *= SPEED_SCALAR;

                    double targetVelocityTicks = (targetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;

                    leftVelocity = targetVelocityTicks;
                    rightVelocity = targetVelocityTicks;

                    // Boost left motor if over 120 inches (Same logic)
                    if (currentDist > 120.0) {
                        leftVelocity *= 1.80;
                    }

                    LflywheelMotor.setVelocity(leftVelocity);
                    RflywheelMotor.setVelocity(rightVelocity);
                }
            } else {
                // Optional: Ensure servo stays low if target lost, or just hold last pos.
                // Resetting to low is safer to prevent damage if robot drives near walls.
                hoodServo.setPosition(SERVO_LOW_POS);
            }

            // --- TELEMETRY ---
            double LactualRPM = (LflywheelMotor.getVelocity() * 60) / FLYWHEEL_TICKS_PER_REV;
            double RactualRPM = (RflywheelMotor.getVelocity() * 60) / FLYWHEEL_TICKS_PER_REV;

            telemetry.addData("1. Distance", "%.2f in", currentDist);
            telemetry.addData("2. Servo Pos", "%s", servoStatus);
            telemetry.addData("3. Left RPM", "Target: %.0f | Actual: %.0f", targetRPM, LactualRPM);
            telemetry.addData("4. Right RPM", "Target: %.0f | Actual: %.0f", targetRPM, RactualRPM);
            telemetry.update();
        }
        limelight.stop();
    }

    // Exact physics method from SmartShooter
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

    private static final double G = 386.1;
    private class ShotData {
        public double launchAngleDegrees;
        public double launchVelocityInchesPerSec;
        public boolean isPossible;
    }
}