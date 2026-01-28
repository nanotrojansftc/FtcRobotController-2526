package org.firstinspires.ftc.teamcode.NanoTrojans.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp(name = "Smart Shooter - 1000ms Memory", group = "Competition")
public class SmartShooterWithServo extends LinearOpMode {

    // --- HARDWARE ---
    private Limelight3A limelight;
    private DcMotorEx LflywheelMotor, RflywheelMotor;
    private Servo hoodServo;

    // --- LOGIC VARIABLES ---
    private ElapsedTime signalLossTimer = new ElapsedTime();
    private double lastTargetRPM = 0;       // For display
    private double lastTargetLeftTicks = 0; // For motor control
    private double lastTargetRightTicks = 0;
    private double lastServoPos = 0.5;
    private boolean hasTargetMemory = false;

    // --- CONSTANTS ---
    // UPDATED: Increased to 1000ms (1 second) to handle heavy vibration
    final double FLICKER_THRESHOLD_MS = 1000;

    final double SERVO_LOW_POS  = 0.5;
    final double SERVO_HIGH_POS = 0.58;
    final double SERVO_THRESHOLD_DIST = 90.0;

    // --- MEASUREMENTS ---
    final double CAMERA_HEIGHT_INCHES = 16.25;
    final double TAG_HEIGHT_INCHES    = 29.5;
    final double MOUNT_ANGLE_DEGREES  = 10.6;
    final double SHOOTER_HEIGHT       = 17.7;
    final double BASKET_HEIGHT        = 43.0;
    final double ENTRY_ANGLE          = -45.0;
    final double G = 386.1;

    final double FLYWHEEL_TICKS_PER_REV = 28.0;
    final double FLYWHEEL_RADIUS = 1.89;
    final double SPEED_SCALAR = 2.8;

    @Override
    public void runOpMode() {
        // --- 0. BATTERY CHECK ---
        // Get the battery voltage sensor
        com.qualcomm.robotcore.hardware.VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // 1. INIT HARDWARE
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // 2. INIT FLYWHEELS (TUNED FOR CONSISTENCY)
        LflywheelMotor = hardwareMap.get(DcMotorEx.class, "lgun");
        LflywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // OLD VALUES: P=100 (Softer), F=16.45 (Fixes overshoot)
        // NEW VALUES: P=70 (Softer), F=13.5 (Fixes overshoot)
        PIDFCoefficients tunedPIDF = new PIDFCoefficients(70, 0, 1.5, 13.5);

        LflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tunedPIDF);

        RflywheelMotor = hardwareMap.get(DcMotorEx.class, "rgun");
        RflywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tunedPIDF);


        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setPosition(SERVO_LOW_POS);

        // --- INIT LOOP (Display Voltage While Waiting) ---
        while (!isStarted() && !isStopRequested()) {
            double voltage = batteryVoltageSensor.getVoltage();

            telemetry.addData("Status", "Initialized & Ready.");
            telemetry.addData("Battery Voltage", "%.2f V", voltage);

            // Warning if battery is too low for consistent shooting
            if (voltage < 12.5) {
                telemetry.addData("WARNING", "BATTERY LOW! SWAP NOW.");
            } else {
                telemetry.addData("Battery Status", "GOOD");
            }

            telemetry.update();
        }

        // waitForStart() is handled by the loop above now
        signalLossTimer.reset();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            boolean validResult = (result != null && result.isValid());
            double currentDist = 0;
            int tagID = -1;

            if (validResult) {
                // --- TARGET FOUND (REFRESH MEMORY) ---
                signalLossTimer.reset();
                hasTargetMemory = true;

                // Get Tag ID
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (!fiducials.isEmpty()) tagID = fiducials.get(0).getFiducialId();

                // Calculate Distance
                double ty = result.getTy();
                double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEGREES + ty);
                currentDist = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

                ShotData shot = calculateShot(currentDist, BASKET_HEIGHT - SHOOTER_HEIGHT, ENTRY_ANGLE);

                if (shot.isPossible) {
                    // Update Servo Memory
                    if (currentDist < SERVO_THRESHOLD_DIST) lastServoPos = SERVO_LOW_POS;
                    else lastServoPos = SERVO_HIGH_POS;

                    // Update RPM Memory
                    double targetRPM = (shot.launchVelocityInchesPerSec * 60) / (2 * Math.PI * FLYWHEEL_RADIUS);
                    targetRPM *= SPEED_SCALAR;
                    lastTargetRPM = targetRPM; // Store for telemetry

                    // Calculate Motor Ticks
                    double ticks = (targetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;
                    lastTargetLeftTicks = ticks;
                    lastTargetRightTicks = ticks;

                    if (currentDist > 120.0) lastTargetLeftTicks *= 1.80;

                    // Apply
                    hoodServo.setPosition(lastServoPos);
                    LflywheelMotor.setVelocity(lastTargetLeftTicks);
                    RflywheelMotor.setVelocity(lastTargetRightTicks);
                }
            }
            else {
                // --- SIGNAL LOST (USE MEMORY) ---
                if (signalLossTimer.milliseconds() < FLICKER_THRESHOLD_MS && hasTargetMemory) {
                    // HOLD LAST VALUES
                    hoodServo.setPosition(lastServoPos);
                    LflywheelMotor.setVelocity(lastTargetLeftTicks);
                    RflywheelMotor.setVelocity(lastTargetRightTicks);
                } else {
                    // TIMEOUT EXCEEDED -> RESET
                    hoodServo.setPosition(SERVO_LOW_POS);
                    LflywheelMotor.setVelocity(0);
                    RflywheelMotor.setVelocity(0);
                    hasTargetMemory = false;
                    lastTargetRPM = 0; // Clear for display
                }
            }

            // --- TELEMETRY ---
            // Calculate Real-Time RPM from motor velocity (ticks per second)
            // Formula: (TicksPerSec * 60) / TicksPerRev
            double currentLeftRPM = (LflywheelMotor.getVelocity() * 60) / FLYWHEEL_TICKS_PER_REV;
            double currentRightRPM = (RflywheelMotor.getVelocity() * 60) / FLYWHEEL_TICKS_PER_REV;

            telemetry.addData("--- TARGETING ---", "");
            if (validResult) {
                telemetry.addData("Status", "LOCKED ON (Tag %d)", tagID);
                telemetry.addData("Distance", "%.1f in", currentDist);
            } else if (hasTargetMemory) {
                telemetry.addData("Status", "MEMORY HOLD (%.0f ms left)", (FLICKER_THRESHOLD_MS - signalLossTimer.milliseconds()));
            } else {
                telemetry.addData("Status", "SEARCHING / IDLE");
            }

            telemetry.addData("--- SHOOTER ---", "");
            telemetry.addData("Servo Pos", "%.2f", hoodServo.getPosition());

            // UPDATED: Explicit Target vs Actual comparison
            telemetry.addData("TARGET RPM", "%.0f", lastTargetRPM);
            telemetry.addData("Actual Left RPM", "%.0f", currentLeftRPM);
            telemetry.addData("Actual Right RPM", "%.0f", currentRightRPM);

            telemetry.update();
        }
        limelight.stop();
    }

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