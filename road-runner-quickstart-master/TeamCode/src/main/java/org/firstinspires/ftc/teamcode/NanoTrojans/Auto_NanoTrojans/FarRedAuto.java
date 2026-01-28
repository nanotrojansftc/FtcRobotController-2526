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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/*
 * ==========================================================================================
 * NANO TROJANS - FAR RED AUTONOMOUS (TUNED & COMMENTED)
 * ==========================================================================================
 * Strategy:
 * 1. Init: Scan Obelisk & Inventory.
 * 2. Start: Shoot Pre-load -> Intake Samples (Smart) -> Score -> Repeat.
 * ==========================================================================================
 */

@Autonomous(name = "Far Red Auto - Smart", group = "Competition")
public class FarRedAuto extends LinearOpMode {

    // --- HARDWARE ---
    private Follower follower;
    private Paths paths;
    private DcMotorEx LflywheelMotor, RflywheelMotor;
    private Servo hoodServo;
    private DcMotorEx spindexerMotor;
    private DcMotor intakeMotor;
    private Servo llift, rlift;
    private Limelight3A limelight;

    // --- SENSORS ---
    private ColorSensor intakeSensor;
    private ColorSensor lgunsensor, rgunsensor;

    // --- LOGIC ---
    private int detectedApriltagId = -1;
    private List<String> currentInventory = new ArrayList<>();

    // ==================================================================
    // 🔧 TUNING SECTION: SPINDEXER
    // ==================================================================

    // ADJUST HERE: If spindexer vibrates/shakes, LOWER this (try 0.002)
    final double SPINDEXER_KP = 0.003;

    // ADJUST HERE: If spindexer stalls/stucks, INCREASE this (try 0.09)
    final double SPINDEXER_KF = 0.075;

    // ADJUST HERE: Exact encoder ticks for 1/5th rotation.
    final double TICKS_PER_COMPARTMENT = 250.5;

    private int currentSpindexerSlot = 0;

    // ==================================================================
    // 🔧 TUNING SECTION: SHOOTER
    // ==================================================================

    // ADJUST HERE: Wait time (ms) to stabilize before shooting.
    // Increase if accuracy is bad.
    final long SETTLING_TIME_MS = 300;

    // ADJUST HERE: Allowed error in RPM. Increase if robot waits too long to shoot.
    private final double RPM_TOLERANCE = 150.0;

    // ADJUST HERE: "Fudge Factor" for shot power.
    // > 3.0 = Shoot Harder/Further. < 3.0 = Shoot Softer/Shorter.
    final double SPEED_SCALAR = 3.0;

    // ADJUST HERE: Servo Angles
    final double SERVO_LOW_POS  = 0.5;   // Hood Down
    final double SERVO_HIGH_POS = 0.58;  // Hood Up

    // ADJUST HERE: Distance (inches) to switch Hood Angle
    final double SERVO_THRESHOLD_DIST = 90.0;

    // ==================================================================
    // 🔧 TUNING SECTION: FIELD MEASUREMENTS
    // ==================================================================
    final double CAMERA_HEIGHT_INCHES = 16.25;
    final double TAG_HEIGHT_INCHES    = 29.5;
    final double MOUNT_ANGLE_DEGREES  = 10.6;  // ADJUST HERE: Tilt of Limelight
    final double SHOOTER_HEIGHT = 17.7;
    final double BASKET_HEIGHT  = 43.0;
    final double ENTRY_ANGLE    = -45;
    final double FLYWHEEL_RADIUS = 1.89;

    // INTERNAL VARIABLES
    private double targetLeftVel = 0;
    private double targetRightVel = 0;
    private double lastServoPos = 0.5;
    private ElapsedTime signalLossTimer = new ElapsedTime();
    private boolean hasTargetMemory = false;
    final double FLYWHEEL_TICKS_PER_REV = 28.0;
    private static final double G = 386.1;

    // STATE MACHINE
    private enum IntakeState { INIT, INTAKING, DETECTED_WAIT, INDEXING, FULL }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        follower = Constants.createFollower(hardwareMap);
        // ADJUST HERE: Starting Pose (X, Y, Heading Radians)
        follower.setStartingPose(new Pose(87.500, 8.000, Math.toRadians(90)));
        paths = new Paths(follower);

        telemetry.addData("Status", "Ready. Scan Obelisk NOW.");
        telemetry.update();

        // --------------------------------------------------------
        // INIT LOOP (Pre-Match Scanning)
        // --------------------------------------------------------
        while (!isStarted() && !isStopRequested()) {
            int id = readObeliskTagInstant();
            if (id != -1) {
                detectedApriltagId = id;
                telemetry.addData("LOCKED ON", "Tag ID: %d", detectedApriltagId);
            } else {
                telemetry.addData("SEARCHING", "Point Camera at Obelisk...");
            }

            scanInventory();
            telemetry.addData("Inventory", currentInventory.toString());
            telemetry.update();

            autoAimAndSpinUp();
        }

        if (opModeIsActive()) {

            // --- SEGMENT 1: PRE-LOAD ---
            follower.followPath(paths.Path1, true);
            driveWithActions(false);

            // ADJUST HERE: Shooting Position 1
            follower.holdPoint(new Pose(87.5, 27.25, Math.toRadians(64)));
            smartSleep(SETTLING_TIME_MS);
            performSmartShot();

            // --- SEGMENT 2: MOVE TO SAMPLE ---
            follower.followPath(paths.Path2, true);
            driveWithActions(false);

            // --- SEGMENT 3: SMART INTAKE ---
            // ADJUST HERE: Intake Drive Speed (0.5 = 50%)
            follower.setMaxPower(0.5);
            follower.followPath(paths.Path3, true);

            // ADJUST HERE: Intake Timeout (4000ms = 4s)
            runSmartIntakeCycle(4000);

            follower.setMaxPower(1.0);

            // --- SEGMENT 4: SCORE ---
            follower.followPath(paths.Path4, true);
            driveWithActions(false);

            // ADJUST HERE: Shooting Position 2
            follower.holdPoint(new Pose(86.75, 27.5, Math.toRadians(64)));
            smartSleep(SETTLING_TIME_MS);
            performSmartShot();

            // --- SEGMENT 5: MOVE OUT ---
            follower.followPath(paths.Path5, true);
            driveWithActions(false);

            // --- SEGMENT 6: SMART INTAKE ---
            follower.setMaxPower(0.5);
            follower.followPath(paths.Path6, true);

            // ADJUST HERE: Intake Timeout (4000ms)
            runSmartIntakeCycle(4000);

            follower.setMaxPower(1.0);

            // --- SEGMENT 7: SCORE ---
            follower.followPath(paths.Path7, true);
            driveWithActions(false);

            // ADJUST HERE: Shooting Position 3
            follower.holdPoint(new Pose(86.75, 27.5, Math.toRadians(64)));
            smartSleep(SETTLING_TIME_MS);
            performSmartShot();

            stopShooter();
        }
    }

    // =========================================================================
    //   SMART INTAKE STATE MACHINE
    // =========================================================================
    private void runSmartIntakeCycle(double timeoutMs) {
        ElapsedTime timer = new ElapsedTime();
        IntakeState state = IntakeState.INIT;
        ElapsedTime stateTimer = new ElapsedTime();

        while (opModeIsActive() && timer.milliseconds() < timeoutMs && state != IntakeState.FULL) {
            follower.update();
            autoAimAndSpinUp();

            switch (state) {
                case INIT:
                    // Safety: Drop lifts
                    llift.setPosition(SERVO_LOW_POS);
                    rlift.setPosition(SERVO_LOW_POS);
                    intakeMotor.setPower(1.0);
                    state = IntakeState.INTAKING;
                    break;

                case INTAKING:
                    if (currentInventory.size() >= 3) {
                        state = IntakeState.FULL;
                    }
                    // ADJUST HERE: Sensor Threshold (200 is typical for Alpha)
                    else if (intakeSensor.alpha() > 200) {
                        stateTimer.reset();
                        state = IntakeState.DETECTED_WAIT;
                    }
                    break;

                case DETECTED_WAIT:
                    // ADJUST HERE: Wait time for pixel to enter fully (100ms)
                    if (stateTimer.milliseconds() > 100) {
                        state = IntakeState.INDEXING;
                    }
                    break;

                case INDEXING:
                    rotateSpindexerOneSlot();
                    scanInventory();
                    telemetry.addData("Captured", currentInventory.toString());
                    telemetry.update();
                    state = IntakeState.INTAKING;
                    break;

                case FULL:
                    break;
            }
        }
        intakeMotor.setPower(0);
        setSafeSpindexerPower(0);
    }

    private void rotateSpindexerOneSlot() {
        currentSpindexerSlot++;
        double target = currentSpindexerSlot * TICKS_PER_COMPARTMENT;
        ElapsedTime safety = new ElapsedTime();

        // ADJUST HERE: Max time to try rotating (800ms)
        while(opModeIsActive() && safety.milliseconds() < 800) {
            double current = spindexerMotor.getCurrentPosition();
            // ADJUST HERE: Position Tolerance (15 ticks)
            if (Math.abs(target - current) < 15) break;

            double error = target - current;
            double power = (error * SPINDEXER_KP);

            // ADJUST HERE: Max Power (0.6)
            power = Math.max(-0.6, Math.min(0.6, power));
            setSafeSpindexerPower(power);
        }
        setSafeSpindexerPower(0);
    }

    // =========================================================================
    //   INVENTORY & SENSOR LOGIC
    // =========================================================================
    private void scanInventory() {
        currentInventory.clear();

        // GUN SENSORS (Back)
        if (isColor(lgunsensor, "PURPLE")) currentInventory.add("Purple");
        else if (isColor(lgunsensor, "GREEN")) currentInventory.add("Green");

        if (isColor(rgunsensor, "PURPLE")) currentInventory.add("Purple");
        else if (isColor(rgunsensor, "GREEN")) currentInventory.add("Green");

        // INTAKE SENSOR (Front)
        // ADJUST HERE: Threshold for front sensor (200)
        if (intakeSensor.alpha() > 200) {
            if (intakeSensor.blue() > intakeSensor.red()) currentInventory.add("Purple");
            else currentInventory.add("Green");
        }
    }

    private boolean isColor(ColorSensor sensor, String target) {
        if (target.equals("PURPLE")) {
            return sensor.blue() > sensor.red() && sensor.blue() > sensor.green();
        } else if (target.equals("GREEN")) {
            return sensor.green() > sensor.blue() && sensor.green() > sensor.red();
        }
        return false;
    }

    // =========================================================================
    //   SMART SHOOT LOGIC
    // =========================================================================
    private void performSmartShot() {
        scanInventory();

        int purpleCount = Collections.frequency(currentInventory, "Purple");
        int greenCount = Collections.frequency(currentInventory, "Green");

        // RULE: 3 Items, 2P + 1G
        boolean isFull = currentInventory.size() >= 3;
        boolean isValidComp = (purpleCount >= 2 && greenCount >= 1);

        if (!isFull || !isValidComp) {
            telemetry.addData("Strategy", "BAD LOAD - DUMPING!");
            shootGeneric();
        } else {
            telemetry.addData("Strategy", "MATCHING TAG: " + detectedApriltagId);
            if (detectedApriltagId == 21) shootGPP();
            else if (detectedApriltagId == 22) shootPGP();
            else if (detectedApriltagId == 23) shootPPG();
            else shootPPG();
        }
        telemetry.update();
    }

    private void shootGeneric() {
        waitForFlywheelReady();
        performSingleShotLeft();
        smartSleep(200);
        performSingleShotRight();
        smartSleep(200);
        rotateSpindexerOneSlot();
        smartSleep(200);
        performSingleShotLeft();
        llift.setPosition(1);
        rlift.setPosition(0.005);
    }

    // =========================================================================
    //   UNIVERSAL SAFETY RULE
    // =========================================================================
    private void setSafeSpindexerPower(double power) {
        if (Math.abs(power) > 0.05) {
            // Safety: If Lifts are UP, force them DOWN
            // ADJUST HERE: Safety Tolerance (0.1)
            if (Math.abs(llift.getPosition() - SERVO_LOW_POS) > 0.1 ||
                    Math.abs(rlift.getPosition() - SERVO_LOW_POS) > 0.1) {

                llift.setPosition(SERVO_LOW_POS);
                rlift.setPosition(SERVO_LOW_POS);
            }
        }
        spindexerMotor.setPower(power);
    }

    // =========================================================================
    //   SHOOTING PATTERNS
    // =========================================================================
    private void shootGPP() {
        safeShootDual(true, true);
        smartSleep(250);
        rotateSpindexerOneSlot();
        safeShootPurple();
    }

    private void shootPGP() {
        safeShootDual(true, false);
        smartSleep(250);
        rotateSpindexerOneSlot();
        safeShootPurple();
    }

    private void shootPPG() {
        safeShootDual(true, false);
        smartSleep(250);
        rotateSpindexerOneSlot();
        safeShootGreen();
    }

    private boolean safeShootGreen() {
        performSingleShotLeft();
        return true;
    }

    private boolean safeShootPurple() {
        performSingleShotRight();
        return true;
    }

    private void safeShootDual(boolean leftFirst, boolean isGPP) {
        waitForFlywheelReady();
        if (isGPP) {
            llift.setPosition(0.625); // ADJUST HERE: Left Lift Pos
            smartSleep(200);
            waitForFlywheelReady();
            rlift.setPosition(0.3);   // ADJUST HERE: Right Lift Pos
        } else {
            rlift.setPosition(0.3);
            smartSleep(200);
            waitForFlywheelReady();
            llift.setPosition(0.625);
        }
        smartSleep(300);
        llift.setPosition(1); rlift.setPosition(0.005);
        smartSleep(400);
    }

    private void performSingleShotLeft() {
        waitForFlywheelReady();
        llift.setPosition(0.625);
        smartSleep(300);
        llift.setPosition(1);
        smartSleep(400);
    }

    private void performSingleShotRight() {
        waitForFlywheelReady();
        rlift.setPosition(0.3);
        smartSleep(300);
        rlift.setPosition(0.005);
        smartSleep(400);
    }

    private void stopShooter() {
        LflywheelMotor.setVelocity(0);
        RflywheelMotor.setVelocity(0);
    }

    // =========================================================================
    //   UTILITIES & PHYSICS
    // =========================================================================
    public void smartSleep(long milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            follower.update();
            autoAimAndSpinUp();
        }
    }

    private void waitForFlywheelReady() {
        ElapsedTime timeout = new ElapsedTime();
        // ADJUST HERE: Max wait time for flywheel (0.8s)
        while (opModeIsActive() && timeout.seconds() < 0.8) {
            follower.update();
            autoAimAndSpinUp();

            double currentLeft = LflywheelMotor.getVelocity();
            double currentRight = RflywheelMotor.getVelocity();
            if (Math.abs(currentLeft - targetLeftVel) < RPM_TOLERANCE &&
                    Math.abs(currentRight - targetRightVel) < RPM_TOLERANCE) break;
        }
    }

    private void driveWithActions(boolean intakeOn) {
        // ADJUST HERE: Intake Power while driving (0.75)
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

    private void autoAimAndSpinUp() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            signalLossTimer.reset();
            hasTargetMemory = true;

            double ty = result.getTy();
            double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEGREES + ty);
            double currentDist = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

            double heightY = BASKET_HEIGHT - SHOOTER_HEIGHT;
            ShotData shot = calculateShot(currentDist, heightY, ENTRY_ANGLE);

            if (shot.isPossible) {
                lastServoPos = (currentDist < SERVO_THRESHOLD_DIST) ? SERVO_LOW_POS : SERVO_HIGH_POS;

                double targetRPM = (shot.launchVelocityInchesPerSec * 60) / (2 * Math.PI * FLYWHEEL_RADIUS);
                targetRPM *= SPEED_SCALAR;
                double targetVelocityTicks = (targetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;

                targetLeftVel = targetVelocityTicks;
                targetRightVel = targetVelocityTicks;

                // ADJUST HERE: Long Distance Speed Multiplier (1.80)
                if (currentDist > 120.0) targetLeftVel *= 1.80;

                hoodServo.setPosition(lastServoPos);
                LflywheelMotor.setVelocity(targetLeftVel);
                RflywheelMotor.setVelocity(targetRightVel);
            }
        } else {
            if (signalLossTimer.milliseconds() < 1000 && hasTargetMemory) {
                hoodServo.setPosition(lastServoPos);
                LflywheelMotor.setVelocity(targetLeftVel);
                RflywheelMotor.setVelocity(targetRightVel);
            } else {
                hoodServo.setPosition(SERVO_LOW_POS);
                LflywheelMotor.setVelocity(0);
                RflywheelMotor.setVelocity(0);
                hasTargetMemory = false;
            }
        }
    }

    // =========================================================================
    //   INIT HARDWARE
    // =========================================================================
    private void initHardware() {
        LflywheelMotor = hardwareMap.get(DcMotorEx.class, "lgun");
        LflywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RflywheelMotor = hardwareMap.get(DcMotorEx.class, "rgun");
        RflywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setPosition(SERVO_LOW_POS);

        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        llift = hardwareMap.get(Servo.class, "llift");
        rlift = hardwareMap.get(Servo.class, "rlift");

        // SENSORS
        lgunsensor = hardwareMap.get(ColorSensor.class, "lgunsensor");
        rgunsensor = hardwareMap.get(ColorSensor.class, "rgunsensor");
        intakeSensor = hardwareMap.get(ColorSensor.class, "intake sensor");

        llift.setPosition(1);
        rlift.setPosition(0.005);
    }

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

    // =========================================================================
    //   PATH DEFINITIONS
    // =========================================================================
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;
        public Paths(Follower follower) {
            // ADJUST HERE: All Path Coordinates (X, Y, Heading)

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