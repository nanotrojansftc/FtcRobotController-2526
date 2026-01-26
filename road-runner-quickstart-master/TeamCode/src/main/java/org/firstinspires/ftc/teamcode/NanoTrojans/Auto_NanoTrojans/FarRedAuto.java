package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;
import java.util.List;

@Autonomous(name = "Far Red Auto", group = "Competition")
public class FarRedAuto extends LinearOpMode {

    private Follower follower;
    private Paths paths;
    private DcMotorEx LflywheelMotor, RflywheelMotor, hoodMotor;
    private DcMotor intakeMotor;
    private CRServo fspin, rspin, lspin;
    private Servo llift, rlift;
    private Limelight3A limelight;
    private colorsensors bench;
    private ColorSensor intakeSensor;

    private int carouselIndex = 0;
    private int detectedApriltagId = -1;

    // --- NEW VARIABLES (Dual Motor RPM Check) ---
    private double targetLeftVel = 0;
    private double targetRightVel = 0;
    // Tolerance: If speed is within 150 ticks of target, it shoots.
    private final double RPM_TOLERANCE = 150.0;

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

    private boolean isRotating = false;
    private ElapsedTime rotationTimer = new ElapsedTime();
    private final double ROTATION_TIME_MS = 410;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(87.500, 8.000, Math.toRadians(90)));
        paths = new Paths(follower);

        telemetry.addData("Status", "Initialized. Dual RPM Check Active.");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            autoAimAndSpinUp();
            int id = readObeliskTagInstant();
            if (id != -1) detectedApriltagId = id;
            telemetry.addData("LOCKED ON", "Tag ID: %d", detectedApriltagId);
            telemetry.update();
        }

        if (opModeIsActive()) {
            // --- SEGMENT 1 ---
            follower.followPath(paths.Path1, true);
            driveWithActions(false);
            follower.holdPoint(new Pose(87.5, 27.25, Math.toRadians(64)));
            performSmartShot();

            // --- SEGMENT 2 ---
            follower.followPath(paths.Path2, true);
            driveWithActions(false);

            // --- SEGMENT 3 ---
            follower.setMaxPower(0.5);
            follower.followPath(paths.Path3, true);
            driveWithIntakeAndSpindex();
            follower.setMaxPower(1.0);

            // --- SEGMENT 4 ---
            follower.followPath(paths.Path4, true);
            driveWithActions(false);
            follower.holdPoint(new Pose(86.75, 27.5, Math.toRadians(64)));
            performSmartShot();

            // --- SEGMENT 5 ---
            follower.followPath(paths.Path5, true);
            driveWithActions(false);

            // --- SEGMENT 6 ---
            follower.setMaxPower(0.5);
            follower.followPath(paths.Path6, true);
            driveWithIntakeAndSpindex();
            follower.setMaxPower(1.0);

            // --- SEGMENT 7 ---
            follower.followPath(paths.Path7, true);
            driveWithActions(false);
            follower.holdPoint(new Pose(86.75, 27.5, Math.toRadians(64)));
            performSmartShot();

            stopShooter();
        }
    }

    // =========================================================================
    //   HELPER: Wait For Dual RPM
    // =========================================================================

    private void waitForFlywheelReady() {
        ElapsedTime timeout = new ElapsedTime();
        // Wait max 1.0 second
        while (opModeIsActive() && timeout.seconds() < 1.0) {
            follower.update();
            autoAimAndSpinUp();

            // Read ACTUAL velocities of both motors
            double currentLeft = LflywheelMotor.getVelocity();
            double currentRight = RflywheelMotor.getVelocity();

            // Check if BOTH are within tolerance of their specific targets
            boolean leftReady = Math.abs(currentLeft - targetLeftVel) < RPM_TOLERANCE;
            boolean rightReady = Math.abs(currentRight - targetRightVel) < RPM_TOLERANCE;

            if (leftReady && rightReady) {
                break; // Both are ready! Fire!
            }
        }
    }

    public void smartSleep(long milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            follower.update();
            autoAimAndSpinUp();
        }
    }

    // =========================================================================
    //   DRIVING LOOPS
    // =========================================================================

    private void driveWithActions(boolean intakeOn) {
        if(intakeOn) intakeMotor.setPower(0.75);
        else intakeMotor.setPower(0);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            autoAimAndSpinUp();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().drawImage("/images/field.png", 0, 0, 144, 144);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        intakeMotor.setPower(0);
    }

    private void driveWithIntakeAndSpindex() {
        intakeMotor.setPower(0.75);
        isRotating = false;

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            autoAimAndSpinUp();

            if (isRotating) {
                if (rotationTimer.milliseconds() > ROTATION_TIME_MS) {
                    stopCarousel();
                    isRotating = false;
                    carouselIndex++;
                    if (carouselIndex > 2) carouselIndex = 0;
                }
            } else {
                boolean chamberFull = (isColor(bench.left, colorsensors.DetectedColor.GREEN) ||
                        isColor(bench.left, colorsensors.DetectedColor.PURPLE)) &&
                        (isColor(bench.right, colorsensors.DetectedColor.GREEN) ||
                                isColor(bench.right, colorsensors.DetectedColor.PURPLE));

                double distanceMM = ((DistanceSensor) intakeSensor).getDistance(DistanceUnit.MM);
                boolean ballAtIntake = distanceMM < 40;

                if (ballAtIntake && !chamberFull) {
                    startCarouselRotation();
                }
            }
            telemetry.addData("INTAKE", "Active");
            telemetry.update();
        }
        intakeMotor.setPower(0);
        stopCarousel();
    }

    private void startCarouselRotation() {
        isRotating = true;
        rotationTimer.reset();
        if (carouselIndex == 0) fspin.setPower(-1);
        else if (carouselIndex == 1) rspin.setPower(-1);
        else if (carouselIndex == 2) lspin.setPower(-1);
    }

    private void stopCarousel() {
        fspin.setPower(0);
        rspin.setPower(0);
        lspin.setPower(0);
    }

    // =========================================================================
    //   SHOOTING LOGIC (Dual RPM + No Extra Delay)
    // =========================================================================

    private void performSmartShot() {
        if (detectedApriltagId == 21) shootGPP();
        else if (detectedApriltagId == 22) shootPGP();
        else if (detectedApriltagId == 23) shootPPG();
        else shootPPG();
    }

    private void shootGPP() {
        telemetry.addData("Shooting", "GPP Sequence");
        telemetry.update();
        waitForFlywheelReady();

        if (isColor(bench.left, colorsensors.DetectedColor.GREEN) &&
                isColor(bench.right, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(true, true);
            smartSleep(250);
            rotatePurpleIn();
            safeShootPurple();
        }
        else if (isColor(bench.right, colorsensors.DetectedColor.GREEN) &&
                isColor(bench.left, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(false, true);
            smartSleep(250);
            rotatePurpleIn();
            safeShootPurple();
        }
        else {
            rotateGreenIn();
            if (!safeShootGreen()) { rotateGreenIn(); safeShootGreen(); }
            smartSleep(250);
            if (!safeShootPurple()) { rotatePurpleIn(); safeShootPurple(); }
            smartSleep(250);
            if (!safeShootPurple()) { rotatePurpleIn(); safeShootPurple(); }
        }
    }

    private void shootPGP() {
        telemetry.addData("Shooting", "PGP Sequence");
        telemetry.update();
        waitForFlywheelReady();

        if (isColor(bench.left, colorsensors.DetectedColor.GREEN) &&
                isColor(bench.right, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(true, false);
            smartSleep(250);
            rotatePurpleIn();
            safeShootPurple();
        } else if (isColor(bench.right, colorsensors.DetectedColor.GREEN) &&
                isColor(bench.left, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(false, false);
            smartSleep(250);
            rotatePurpleIn();
            safeShootPurple();
        } else {
            rotatePurpleIn();
            if (!safeShootPurple()) { rotatePurpleIn(); safeShootPurple(); }
            smartSleep(250);
            if (!safeShootGreen()) { rotateGreenIn(); safeShootGreen(); }
            smartSleep(250);
            if (!safeShootPurple()) { rotatePurpleIn(); safeShootPurple(); }
        }
    }

    private void shootPPG() {
        telemetry.addData("Shooting", "PPG Sequence");
        telemetry.update();
        waitForFlywheelReady();

        if (isColor(bench.left, colorsensors.DetectedColor.PURPLE) &&
                isColor(bench.right, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(true, false);
            smartSleep(250);
            rotateGreenIn();
            safeShootGreen();
        } else if (isColor(bench.right, colorsensors.DetectedColor.PURPLE) &&
                isColor(bench.left, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(false, false);
            smartSleep(250);
            rotateGreenIn();
            safeShootGreen();
        } else {
            rotatePurpleIn();
            if (!safeShootPurple()) { rotatePurpleIn(); safeShootPurple(); }
            smartSleep(250);
            if (!safeShootPurple()) { rotatePurpleIn(); safeShootPurple(); }
            smartSleep(250);
            if (!safeShootGreen()) { rotateGreenIn(); safeShootGreen(); }
        }
    }

    // =========================================================================
    //   HELPER ACTIONS (Updated Timings)
    // =========================================================================

    private boolean safeShootGreen() {
        if (isColor(bench.left, colorsensors.DetectedColor.GREEN)) {
            performSingleShotLeft(); return true;
        } else if (isColor(bench.right, colorsensors.DetectedColor.GREEN)) {
            performSingleShotRight(); return true;
        }
        return false;
    }

    private boolean safeShootPurple() {
        if (isColor(bench.left, colorsensors.DetectedColor.PURPLE)) {
            performSingleShotLeft(); return true;
        } else if (isColor(bench.right, colorsensors.DetectedColor.PURPLE)) {
            performSingleShotRight(); return true;
        }
        return false;
    }

    private void safeShootDual(boolean leftFirst, boolean isGPP) {
        waitForFlywheelReady(); // Check both motors

        if (isGPP) {
            llift.setPosition(0.625);
            smartSleep(200); // Short wait for servo travel
            waitForFlywheelReady(); // Check both again
            rlift.setPosition(0.3);
        } else {
            rlift.setPosition(0.3);
            smartSleep(200);
            waitForFlywheelReady();
            llift.setPosition(0.625);
        }

        smartSleep(300); // Wait for ball to physically leave
        llift.setPosition(1); rlift.setPosition(0.005);
        // REMOVED: The extra 400ms delay here
    }

    private void performSingleShotLeft() {
        waitForFlywheelReady(); // Check both
        llift.setPosition(0.625);
        smartSleep(300);
        llift.setPosition(1);
        // REMOVED: The extra 400ms delay here
    }

    private void performSingleShotRight() {
        waitForFlywheelReady(); // Check both
        rlift.setPosition(0.3);
        smartSleep(300);
        rlift.setPosition(0.005);
        // REMOVED: The extra 400ms delay here
    }

    private boolean rotateGreenIn() {
        rotate(); smartSleep(400);
        if (!greenInChamber()) { rotate(); smartSleep(400); }
        if (!greenInChamber()) { rotate(); smartSleep(400); }
        return greenInChamber();
    }

    private boolean rotatePurpleIn() {
        rotate(); smartSleep(400);
        if (!purpleInChamber()) { rotate(); smartSleep(400); }
        if (!purpleInChamber()) { rotate(); smartSleep(400); }
        return purpleInChamber();
    }

    private void rotate() {
        autoAimAndSpinUp();
        long sleepTime = (long) ROTATION_TIME_MS;

        if (carouselIndex == 2) {
            carouselIndex = 0;
            lspin.setPower(-1);
            smartSleep(sleepTime);
            lspin.setPower(0);
        } else if (carouselIndex == 1) {
            carouselIndex += 1;
            rspin.setPower(-1);
            smartSleep(sleepTime);
            rspin.setPower(0);
        } else if (carouselIndex == 0) {
            carouselIndex += 1;
            fspin.setPower(-1);
            smartSleep(sleepTime);
            fspin.setPower(0);
        }
    }

    // =========================================================================
    //   UTILITIES (Updated to save both targets)
    // =========================================================================

    private int readObeliskTagInstant() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tags) {
                int id = tag.getFiducialId();
                if (id >= 21 && id <= 23) return id;
            }
        }
        return -1;
    }

    private void autoAimAndSpinUp() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEGREES + ty);
            double currentDist = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);
            double heightY = BASKET_HEIGHT - SHOOTER_HEIGHT;
            ShotData shot = calculateShot(currentDist, heightY, ENTRY_ANGLE);

            if (shot.isPossible) {
                double physicsAngle = shot.launchAngleDegrees;
                double invertedAngle = MAX_HOOD_DEGREES - physicsAngle;
                int targetTicks = (int) Math.round(10 * invertedAngle * HOOD_TICKS_PER_DEGREE);
                targetTicks = Math.max(0, Math.min((int)(MAX_HOOD_DEGREES * HOOD_TICKS_PER_DEGREE), targetTicks));
                hoodMotor.setTargetPosition(targetTicks);
                hoodMotor.setPower(1.0);

                double targetRPM = (shot.launchVelocityInchesPerSec * 60) / (2 * Math.PI * FLYWHEEL_RADIUS);
                targetRPM *= SPEED_SCALAR;
                double targetVelocityTicks = (targetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;

                double leftVel = targetVelocityTicks;
                if (currentDist > 120.0) leftVel *= 1.75;

                // --- SAVE TARGETS FOR THE CHECKER ---
                targetLeftVel = leftVel;
                targetRightVel = targetVelocityTicks;

                LflywheelMotor.setVelocity(leftVel);
                RflywheelMotor.setVelocity(targetVelocityTicks);
            }
        }
    }

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
        intakeMotor.setPower(0);
    }

    private void initHardware() {
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

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        fspin = hardwareMap.crservo.get("fspin");
        rspin = hardwareMap.crservo.get("rspin");
        lspin = hardwareMap.crservo.get("lspin");
        llift = hardwareMap.get(Servo.class, "llift");
        rlift = hardwareMap.get(Servo.class, "rlift");

        bench = new colorsensors();
        bench.init(hardwareMap);

        intakeSensor = hardwareMap.get(ColorSensor.class, "intake sensor");

        llift.setPosition(1);
        rlift.setPosition(0.005);
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
        if (denominator <= 0) data.isPossible = false;
        else {
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

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(87.500, 8.000), new Pose(86.75, 28 )))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(64))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(86.75, 27.5), new Pose(104.000, 45.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(64), Math.toRadians(-1))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(104.000, 43.000), new Pose(145.000, 45.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(-1))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(145.000, 43.000), new Pose(86.75, 28)))
                    .setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(64))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(86.75, 27.5), new Pose(104.000, 65.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(64), Math.toRadians(-1))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(104.000, 65.000), new Pose(145.000, 65.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(-1))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(145.000, 65.000), new Pose(86.75, 28)))
                    .setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(64))
                    .build();
        }
    }
}