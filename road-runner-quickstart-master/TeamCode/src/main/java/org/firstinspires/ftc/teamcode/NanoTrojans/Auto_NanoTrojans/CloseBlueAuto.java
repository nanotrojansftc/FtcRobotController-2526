package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Close Blue Auto Final", group = "Competition")
public class CloseBlueAuto extends LinearOpMode {

    // --- HARDWARE ---
    private Follower follower;
    private Paths paths;

    private DcMotorEx LflywheelMotor, RflywheelMotor, hoodMotor;
    private DcMotor intakeMotor;
    private CRServo fspin, rspin, lspin;
    private Servo llift, rlift;
    private Limelight3A limelight;
    private colorsensors bench;
    private ColorSensor intakeSensor;

    // --- STATE VARIABLES ---
    private int carouselIndex = 0;
    private int detectedApriltagId = -1;

    // --- PHYSICS CONSTANTS ---
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

    // --- ROTATION VARIABLES ---
    private boolean isRotating = false;
    private ElapsedTime rotationTimer = new ElapsedTime();
    private final double ROTATION_TIME_MS = 474.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. INIT
        initHardware();

        // Setup Pedro Pathing
        follower = Constants.createFollower(hardwareMap);

        // --- MIRRORED STARTING POSE ---
        // Red: (122.6, 122.0, 36.4 deg)
        // Blue: (144-122.6, 144-122.0, 36.4+180) -> (21.4, 22.0, 216.4 deg)
        follower.setStartingPose(new Pose(21.400, 22.000, Math.toRadians(216.4)));
        paths = new Paths(follower);

        telemetry.addData("Status", "Initialized (BLUE).");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // --- SEGMENT 1: DRIVE TO OBSERVE (Path 1) ---
            follower.followPath(paths.Path1, true);
            driveWithActions(false);

            // --- END PATH 1 TASKS ---
            // 1. Scan Obelisk Tag (IDs 21-23 are Global)
            int scannedId = readObeliskTagInstant();
            if (scannedId != -1) {
                detectedApriltagId = scannedId;
                telemetry.addData("OBELISK SCAN", "Found Tag: %d", detectedApriltagId);
            } else {
                telemetry.addData("OBELISK SCAN", "No Tag Found (Defaulting)");
            }
            telemetry.update();

            // 2. Start Shooter Motors
            autoAimAndSpinUp();

            // --- SEGMENT 2: PREP TO SHOOT (Path 2) ---
            follower.followPath(paths.Path2, true);
            driveWithActions(false);

            // --- END PATH 2 TASKS ---
            // Auto-calibrate & Shoot Artifacts
            // Mirrored Heading: 45 + 180 = 225 deg
            follower.holdPoint(new Pose(54.000, 44.000, Math.toRadians(225)));
            performSmartShot();

            // --- SEGMENT 3: APPROACH SAMPLE 1 (Path 3) ---
            follower.followPath(paths.Path3, true);
            driveWithActions(false);

            // --- END PATH 3 TASKS ---
            follower.setMaxPower(0.4);

            // --- SEGMENT 4: INTAKE SAMPLE 1 (Path 4) ---
            follower.followPath(paths.Path4, true);
            driveWithIntakeAndSpindex();

            // --- END PATH 4 TASKS ---
            follower.setMaxPower(1.0);
            autoAimAndSpinUp();

            // --- SEGMENT 5: RETURN TO SHOOT (Path 5) ---
            follower.followPath(paths.Path5, true);
            driveWithActions(false);

            // --- END PATH 5 TASKS ---
            follower.holdPoint(new Pose(54.000, 44.000, Math.toRadians(225)));
            performSmartShot();

            // --- SEGMENT 6: APPROACH SAMPLE 2 (Path 6) ---
            follower.followPath(paths.Path6, true);
            driveWithActions(false);

            // --- END PATH 6 TASKS ---
            follower.setMaxPower(0.4);

            // --- SEGMENT 7: INTAKE SAMPLE 2 (Path 7) ---
            follower.followPath(paths.Path7, true);
            driveWithIntakeAndSpindex();

            // --- END PATH 7 TASKS ---
            follower.setMaxPower(1.0);
            autoAimAndSpinUp();

            // --- SEGMENT 8: RETURN TO SHOOT 2 (Path 8) ---
            follower.followPath(paths.Path8, true);
            driveWithActions(false);

            // --- END PATH 8 TASKS ---
            follower.holdPoint(new Pose(54.000, 44.000, Math.toRadians(225)));
            performSmartShot();

            stopShooter();
        }
    }

    // =========================================================================
    //   DRIVING LOOPS & LOGIC
    // =========================================================================

    private void driveWithActions(boolean intakeOn) {
        if(intakeOn) intakeMotor.setPower(0.5);
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
        intakeMotor.setPower(0.5);
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
                telemetry.addData("Intake Dist", "%.1f mm", distanceMM);
            }
            telemetry.addData("INTAKE", "ACTIVE (BLUE)");
            telemetry.update();
        }

        intakeMotor.setPower(0);
        stopCarousel();
    }

    // =========================================================================
    //   SMART SHOOTER LOGIC (Shared)
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
        autoAimAndSpinUp();
        if (isColor(bench.left, colorsensors.DetectedColor.GREEN) &&
                isColor(bench.right, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(true, true); sleep(250); rotatePurpleIn(); safeShootPurple();
        } else if (isColor(bench.right, colorsensors.DetectedColor.GREEN) &&
                isColor(bench.left, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(false, true); sleep(250); rotatePurpleIn(); safeShootPurple();
        } else {
            rotateGreenIn();
            if (!safeShootGreen()) { rotateGreenIn(); safeShootGreen(); }
            sleep(250);
            if (!safeShootPurple()) { rotatePurpleIn(); safeShootPurple(); }
            sleep(250);
            if (!safeShootPurple()) { rotatePurpleIn(); safeShootPurple(); }
        }
    }

    private void shootPGP() {
        telemetry.addData("Shooting", "PGP Sequence");
        telemetry.update();
        autoAimAndSpinUp();
        if (isColor(bench.left, colorsensors.DetectedColor.GREEN) &&
                isColor(bench.right, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(true, false); sleep(250); rotatePurpleIn(); safeShootPurple();
        } else if (isColor(bench.right, colorsensors.DetectedColor.GREEN) &&
                isColor(bench.left, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(false, false); sleep(250); rotatePurpleIn(); safeShootPurple();
        } else {
            rotatePurpleIn();
            if (!safeShootPurple()) { rotatePurpleIn(); safeShootPurple(); }
            sleep(250);
            if (!safeShootGreen()) { rotateGreenIn(); safeShootGreen(); }
            sleep(250);
            if (!safeShootPurple()) { rotatePurpleIn(); safeShootPurple(); }
        }
    }

    private void shootPPG() {
        telemetry.addData("Shooting", "PPG Sequence");
        telemetry.update();
        autoAimAndSpinUp();
        if (isColor(bench.left, colorsensors.DetectedColor.PURPLE) &&
                isColor(bench.right, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(true, false); sleep(250); rotateGreenIn(); safeShootGreen();
        } else if (isColor(bench.right, colorsensors.DetectedColor.PURPLE) &&
                isColor(bench.left, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(false, false); sleep(250); rotateGreenIn(); safeShootGreen();
        } else {
            rotatePurpleIn();
            if (!safeShootPurple()) { rotatePurpleIn(); safeShootPurple(); }
            sleep(250);
            if (!safeShootPurple()) { rotatePurpleIn(); safeShootPurple(); }
            sleep(250);
            if (!safeShootGreen()) { rotateGreenIn(); safeShootGreen(); }
        }
    }

    // --- SHOOTER HELPERS ---

    private boolean safeShootGreen() {
        autoAimAndSpinUp();
        if (isColor(bench.left, colorsensors.DetectedColor.GREEN)) {
            performSingleShotLeft(); return true;
        } else if (isColor(bench.right, colorsensors.DetectedColor.GREEN)) {
            performSingleShotRight(); return true;
        }
        return false;
    }

    private boolean safeShootPurple() {
        autoAimAndSpinUp();
        if (isColor(bench.left, colorsensors.DetectedColor.PURPLE)) {
            performSingleShotLeft(); return true;
        } else if (isColor(bench.right, colorsensors.DetectedColor.PURPLE)) {
            performSingleShotRight(); return true;
        }
        return false;
    }

    private void safeShootDual(boolean leftFirst, boolean isGPP) {
        autoAimAndSpinUp();
        if (isGPP) {
            llift.setPosition(0.625); sleep(600); rlift.setPosition(0.3);
        } else {
            rlift.setPosition(0.3); sleep(500); llift.setPosition(0.625);
        }
        sleep(600);
        llift.setPosition(1); rlift.setPosition(0.005);
    }

    private void performSingleShotLeft() {
        llift.setPosition(0.625); sleep(1000); llift.setPosition(1); sleep(250);
    }

    private void performSingleShotRight() {
        rlift.setPosition(0.3); sleep(800); rlift.setPosition(0.005); sleep(250);
    }

    private boolean rotateGreenIn() {
        rotate(); sleep(400);
        if (!greenInChamber()) { rotate(); sleep(400); }
        if (!greenInChamber()) { rotate(); sleep(400); }
        return greenInChamber();
    }

    private boolean rotatePurpleIn() {
        rotate(); sleep(400);
        if (!purpleInChamber()) { rotate(); sleep(400); }
        if (!purpleInChamber()) { rotate(); sleep(400); }
        return purpleInChamber();
    }

    private void rotate() {
        autoAimAndSpinUp();
        if (carouselIndex == 2) {
            carouselIndex = 0; lspin.setPower(-1); sleep(474); lspin.setPower(0);
        } else if (carouselIndex == 1) {
            carouselIndex += 1; rspin.setPower(-1); sleep(474); rspin.setPower(0);
        } else if (carouselIndex == 0) {
            carouselIndex += 1; fspin.setPower(-1); sleep(474); fspin.setPower(0);
        }
    }

    private void startCarouselRotation() {
        isRotating = true;
        rotationTimer.reset();
        if (carouselIndex == 0) fspin.setPower(-1);
        else if (carouselIndex == 1) rspin.setPower(-1);
        else if (carouselIndex == 2) lspin.setPower(-1);
    }

    private void stopCarousel() {
        fspin.setPower(0); rspin.setPower(0); lspin.setPower(0);
    }

    // =========================================================================
    //   VISION & CALCULATIONS
    // =========================================================================

    private int readObeliskTagInstant() {
        // Obelisk tags (21, 22, 23) are SHARED/GLOBAL.
        // The Obelisk rotates to show one face to the field.
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

                LflywheelMotor.setVelocity(leftVel);
                RflywheelMotor.setVelocity(targetVelocityTicks);
            }
        }
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

    private class ShotData {
        public double launchAngleDegrees;
        public double launchVelocityInchesPerSec;
        public boolean isPossible;
    }

    // =========================================================================
    //   PATH DEFINITIONS (BLUE - MIRRORED)
    // =========================================================================

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;

        public Paths(Follower follower) {

            // Mirror Logic:
            // X_blue = 144 - X_red
            // Y_blue = 144 - Y_red
            // Heading_blue = Heading_red + 180 (normalized)

            // Path 1
            // Red: (122.6, 122.0) -> (90, 100) | Heading 36.4 -> 112
            // Blue: (21.4, 22.0) -> (54, 44)   | Heading 216.4 -> 292 (-68)
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.400, 22.000),
                                    new Pose(54.000, 44.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(216.4), Math.toRadians(292))
                    .build();

            // Path 2
            // Red: (90, 100) -> (90, 100) | Heading 112 -> 45
            // Blue: (54, 44) -> (54, 44)  | Heading 292 -> 225 (-135)
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 44.000),
                                    new Pose(54.000, 44.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(292), Math.toRadians(225))
                    .build();

            // Path 3
            // Red: (90, 100) -> (104, 91)
            // Blue: (54, 44) -> (40, 53)
            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 44.000),
                                    new Pose(40.000, 53.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path 4
            // Red: (104, 91) -> (125, 91)
            // Blue: (40, 53) -> (19, 53)
            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.000, 53.000),
                                    new Pose(19.000, 53.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path 5 (Return)
            // Red: (125, 91) -> (90, 100) | Heading 0 -> 45 | Reversed
            // Blue: (19, 53) -> (54, 44)  | Heading 180 -> 225 | Reversed
            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 53.000),
                                    new Pose(54.000, 44.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                    .setReversed() // Assuming fixed 'no arg' method
                    .build();

            // Path 6
            // Red: (90, 100) -> (104, 66)
            // Blue: (54, 44) -> (40, 78)
            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 44.000),
                                    new Pose(40.000, 78.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path 7
            // Red: (104, 66) -> (125, 66)
            // Blue: (40, 78) -> (19, 78)
            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.000, 78.000),
                                    new Pose(19.000, 78.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path 8 (Return)
            // Red: (125, 66) -> (90, 100) | Heading 0 -> 45
            // Blue: (19, 78) -> (54, 44)  | Heading 180 -> 225
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 78.000),
                                    new Pose(54.000, 44.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                    .build();
        }
    }
}