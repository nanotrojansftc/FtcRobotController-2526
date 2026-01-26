package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor; // CORRECT IMPORT
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;

import java.util.List;

@Autonomous(name = "Auto_SmartSorter_Final", group = "Competition")
public class Auto_SmartSorter extends LinearOpMode {

    // --- HARDWARE ---
    private DcMotorEx LflywheelMotor, RflywheelMotor, hoodMotor;
    private Limelight3A limelight;
    private CRServo fspin, rspin, lspin;
    private Servo llift, rlift;
    private colorsensors bench;

    // --- STATE VARIABLES ---
    private int carouselIndex = 0;
    private int detectedApriltagId = -1;

    // --- SMART SHOOTER CONSTANTS ---
    final double CAMERA_HEIGHT_INCHES = 16.25;
    final double TAG_HEIGHT_INCHES    = 29.5;
    final double MOUNT_ANGLE_DEGREES  = 10.6;
    final double SHOOTER_HEIGHT = 17.7;
    final double BASKET_HEIGHT  = 43.0;
    final double ENTRY_ANGLE    = -45;
    final double MAX_HOOD_DEGREES = 60.0;
    final double HOOD_TICKS_PER_DEGREE = 3.958;
    final double FLYWHEEL_TICKS_PER_REV = 28.0;
    final double FLYWHEEL_RADIUS = 1.89;
    final double SPEED_SCALAR = 3;
    private static final double G = 386.1;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. HARDWARE INIT
        initHardware();

        telemetry.addData("Status", "Ready. Press Start.");
        telemetry.addData("Diagnostics", "Sensors will be live during scan.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // 2. SCANNING PHASE
            // Loops for 3 seconds looking for Obelisk, while spinning up and showing sensor data
            detectedApriltagId = readObeliskTag(3.0);

            // 3. EXECUTE PATTERN
            if (detectedApriltagId == 21) {
                shootGPP();
            } else if (detectedApriltagId == 22) {
                shootPGP();
            } else if (detectedApriltagId == 23) {
                shootPPG();
            } else {
                telemetry.addData("ERROR", "No Tag Found. Defaulting to PPG.");
                telemetry.update();
                shootPPG();
            }
        }

        stopShooter();
    }

    // =========================================================================
    //   TELEMETRY & SCANNING
    // =========================================================================

    private int readObeliskTag(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        int foundId = -1;

        while (opModeIsActive() && timer.seconds() < seconds && foundId == -1) {
            // 1. Spin Up (Physics)
            autoAimAndSpinUp();

            // 2. Limelight Data
            LLResult result = limelight.getLatestResult();

            // --- LIVE TELEMETRY BLOCK ---
            telemetry.addData(">>> MODE", "SCANNING (Time Left: %.1fs)", seconds - timer.seconds());

            // A. AprilTags
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                telemetry.addData("Tags Visible", tags.size());

                for (LLResultTypes.FiducialResult tag : tags) {
                    int id = tag.getFiducialId();

                    // Display tag info (removed getConfidence call that caused crash)
                    telemetry.addData("Seen Tag ID", id);

                    // Check if it's the Obelisk
                    if (id >= 21 && id <= 23) {
                        foundId = id;
                        telemetry.addData(">>> LOCKED ON <<<", "OBELISK ID: %d", foundId);
                    }
                }
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            // B. Light/Color Sensors (Using bench.left/right directly)
            telemetry.addData("--- SENSORS ---", "");
            // Note: If bench.left/right are NormalizedColorSensors, we must treat them as such
            if(bench != null) {
                telemetry.addData("Left Sensor", bench.detectByHue(bench.left, telemetry));
                telemetry.addData("Right Sensor", bench.detectByHue(bench.right, telemetry));
            }

            telemetry.update();
        }
        return foundId;
    }

    private void logSensorStatus(String stage) {
        String lColor = bench.detectByHue(bench.left, telemetry).toString();
        String rColor = bench.detectByHue(bench.right, telemetry).toString();

        telemetry.addData(">>> STATUS", stage);
        telemetry.addData("Left Chamber", lColor);
        telemetry.addData("Right Chamber", rColor);
        telemetry.update();
    }

    // =========================================================================
    //   LOGIC PATTERNS
    // =========================================================================

    private void shootGPP() {
        telemetry.addData("Pattern", "Executing GPP (Tag 21)");
        logSensorStatus("Start GPP Sequence");

        autoAimAndSpinUp();

        // Using helper isColor with NormalizedColorSensor type
        if (isColor(bench.left, colorsensors.DetectedColor.GREEN) &&
                isColor(bench.right, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(true, true);
            sleep(250);
            rotatePurpleIn();
            safeShootPurple();
        }
        else if (isColor(bench.right, colorsensors.DetectedColor.GREEN) &&
                isColor(bench.left, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(false, true);
            sleep(250);
            rotatePurpleIn();
            safeShootPurple();
        }
        else {
            rotateGreenIn();
            if (!safeShootGreen()) {
                rotateGreenIn();
                safeShootGreen();
            }
            sleep(250);
            if (!safeShootPurple()) {
                rotatePurpleIn();
                safeShootPurple();
            }
            sleep(250);
            if (!safeShootPurple()) {
                rotatePurpleIn();
                safeShootPurple();
            }
        }
    }

    private void shootPGP() {
        telemetry.addData("Pattern", "Executing PGP (Tag 22)");
        logSensorStatus("Start PGP Sequence");
        autoAimAndSpinUp();

        if (isColor(bench.left, colorsensors.DetectedColor.GREEN) &&
                isColor(bench.right, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(true, false);
            sleep(250);
            rotatePurpleIn();
            safeShootPurple();
        } else if (isColor(bench.right, colorsensors.DetectedColor.GREEN) &&
                isColor(bench.left, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(false, false);
            sleep(250);
            rotatePurpleIn();
            safeShootPurple();
        } else {
            rotatePurpleIn();
            if (!safeShootPurple()) {
                rotatePurpleIn();
                safeShootPurple();
            }
            sleep(250);
            if (!safeShootGreen()) {
                rotateGreenIn();
                safeShootGreen();
            }
            sleep(250);
            if (!safeShootPurple()) {
                rotatePurpleIn();
                safeShootPurple();
            }
        }
    }

    private void shootPPG() {
        telemetry.addData("Pattern", "Executing PPG (Tag 23)");
        logSensorStatus("Start PPG Sequence");
        autoAimAndSpinUp();

        if (isColor(bench.left, colorsensors.DetectedColor.PURPLE) &&
                isColor(bench.right, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(true, false);
            sleep(250);
            rotateGreenIn();
            safeShootGreen();
        } else if (isColor(bench.right, colorsensors.DetectedColor.PURPLE) &&
                isColor(bench.left, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(false, false);
            sleep(250);
            rotateGreenIn();
            safeShootGreen();
        } else {
            rotatePurpleIn();
            if (!safeShootPurple()) {
                rotatePurpleIn();
                safeShootPurple();
            }
            sleep(250);
            if (!safeShootPurple()) {
                rotatePurpleIn();
                safeShootPurple();
            }
            sleep(250);
            if (!safeShootGreen()) {
                rotateGreenIn();
                safeShootGreen();
            }
        }
    }

    // =========================================================================
    //   SHOOTING ACTIONS
    // =========================================================================

    private boolean safeShootGreen() {
        autoAimAndSpinUp();
        if (isColor(bench.left, colorsensors.DetectedColor.GREEN)) {
            performSingleShotLeft();
            return true;
        } else if (isColor(bench.right, colorsensors.DetectedColor.GREEN)) {
            performSingleShotRight();
            return true;
        }
        telemetry.addData("Skipped Shot", "Wanted Green, found none.");
        return false;
    }

    private boolean safeShootPurple() {
        autoAimAndSpinUp();
        if (isColor(bench.left, colorsensors.DetectedColor.PURPLE)) {
            performSingleShotLeft();
            return true;
        } else if (isColor(bench.right, colorsensors.DetectedColor.PURPLE)) {
            performSingleShotRight();
            return true;
        }
        telemetry.addData("Skipped Shot", "Wanted Purple, found none.");
        return false;
    }

    private void safeShootDual(boolean leftFirst, boolean isGPP) {
        autoAimAndSpinUp();

        if (isGPP) {
            llift.setPosition(0.625);
            sleep(600);
            rlift.setPosition(0.3);
        } else {
            rlift.setPosition(0.3);
            sleep(500);
            llift.setPosition(0.625);
        }
        sleep(600);

        llift.setPosition(1);
        rlift.setPosition(0.005);
    }

    private void performSingleShotLeft() {
        llift.setPosition(0.625);
        sleep(1000);
        llift.setPosition(1);
        sleep(250);
    }

    private void performSingleShotRight() {
        rlift.setPosition(0.3);
        sleep(800);
        rlift.setPosition(0.005);
        sleep(250);
    }

    // =========================================================================
    //   SORTING HELPERS
    // =========================================================================

    private boolean rotateGreenIn() {
        logSensorStatus("Sorting for GREEN...");
        rotate();
        sleep(400);
        if (!greenInChamber()) {
            rotate();
            sleep(400);
        }
        if (!greenInChamber()) {
            rotate();
            sleep(400);
        }
        logSensorStatus("After Green Sort");
        return greenInChamber();
    }

    private boolean rotatePurpleIn() {
        logSensorStatus("Sorting for PURPLE...");
        rotate();
        sleep(400);
        if (!purpleInChamber()) {
            rotate();
            sleep(400);
        }
        if (!purpleInChamber()) {
            rotate();
            sleep(400);
        }
        logSensorStatus("After Purple Sort");
        return purpleInChamber();
    }

    private void rotate() {
        autoAimAndSpinUp();

        if (carouselIndex == 2) {
            carouselIndex = 0;
            lspin.setPower(-1);
            sleep(474);
            lspin.setPower(0);
        } else if (carouselIndex == 1) {
            carouselIndex += 1;
            rspin.setPower(-1);
            sleep(474);
            rspin.setPower(0);
        } else if (carouselIndex == 0) {
            carouselIndex += 1;
            fspin.setPower(-1);
            sleep(474);
            fspin.setPower(0);
        }
    }

    // =========================================================================
    //   UTILITIES
    // =========================================================================

    private void autoAimAndSpinUp() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEGREES + ty);
            double currentDist = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

            double heightY = BASKET_HEIGHT - SHOOTER_HEIGHT;
            ShotData shot = calculateShot(currentDist, heightY, ENTRY_ANGLE);

            if (shot.isPossible) {
                // Hood
                double physicsAngle = shot.launchAngleDegrees;
                double invertedAngle = MAX_HOOD_DEGREES - physicsAngle;
                int targetTicks = (int) Math.round(10 * invertedAngle * HOOD_TICKS_PER_DEGREE);
                targetTicks = Math.max(0, Math.min((int)(MAX_HOOD_DEGREES * HOOD_TICKS_PER_DEGREE), targetTicks));
                hoodMotor.setTargetPosition(targetTicks);
                hoodMotor.setPower(1.0);

                // Flywheels
                double targetRPM = (shot.launchVelocityInchesPerSec * 60) / (2 * Math.PI * FLYWHEEL_RADIUS);
                targetRPM *= SPEED_SCALAR;
                double targetVelocityTicks = (targetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;

                double leftVel = targetVelocityTicks;
                if (currentDist > 120.0) leftVel *= 1.75;

                LflywheelMotor.setVelocity(leftVel);
                RflywheelMotor.setVelocity(targetVelocityTicks);

                // --- AIM TELEMETRY ---
                telemetry.addData("Aim", "Dist: %.1f | Hood: %d", currentDist, targetTicks);
            }
        }
    }

    // --- FIX: Using NormalizedColorSensor to match your library and error logs ---
    private boolean isColor(NormalizedColorSensor sensor, colorsensors.DetectedColor target) {
        return bench.detectByHue(sensor, telemetry) == target;
    }

    private boolean greenInChamber() {
        return isColor(bench.left, colorsensors.DetectedColor.GREEN) ||
                isColor(bench.right, colorsensors.DetectedColor.GREEN);
    }

    private boolean purpleInChamber() {
        return isColor(bench.left, colorsensors.DetectedColor.PURPLE) ||
                isColor(bench.right, colorsensors.DetectedColor.PURPLE);
    }

    private void stopShooter() {
        LflywheelMotor.setVelocity(0);
        RflywheelMotor.setVelocity(0);
    }

    private void initHardware() {
        // Motors
        LflywheelMotor = hardwareMap.get(DcMotorEx.class, "lgun");
        LflywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 0, 1.5, 16.45));

        RflywheelMotor = hardwareMap.get(DcMotorEx.class, "rgun");
        RflywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 0, 1.5, 16.45));

        hoodMotor = hardwareMap.get(DcMotorEx.class, "hood");
        hoodMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hoodMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hoodMotor.setTargetPosition(0);
        hoodMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hoodMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        // Servos
        fspin = hardwareMap.crservo.get("fspin");
        rspin = hardwareMap.crservo.get("rspin");
        lspin = hardwareMap.crservo.get("lspin");
        llift = hardwareMap.get(Servo.class, "llift");
        rlift = hardwareMap.get(Servo.class, "rlift");

        // Sensors
        bench = new colorsensors();
        bench.init(hardwareMap);

        // Default positions
        llift.setPosition(1);
        rlift.setPosition(0.005);
    }

    // Physics Math
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