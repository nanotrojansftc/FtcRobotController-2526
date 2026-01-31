package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans; // Defines where this file lives in the project

// --- IMPORTS ---
// Imports allow us to use code libraries written by FIRST, Google, or other teams.

// Dashboard: Allows us to see the robot's position on a laptop web browser.

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
import org.firstinspires.ftc.teamcode.NanoTrojans.TeleOp_NanoTrojans.TeleOpAutomation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

// @Autonomous: Registers this program so it appears on the Driver Station phone.
// group="Competition": Organizes it into a specific folder.
@Autonomous(name = "Close Red Auto Final2", group = "Competition")
public class CloseRedAuto2 extends LinearOpMode {

    // --- HARDWARE OBJECTS ---
    private Follower follower; // The "Driver" (Pedro Pathing)
    private Paths paths; // The "Map" (List of coordinates)
    //private CRServo fspin, rspin, lspin; // Spindex (Carousel) servos
    private DcMotorEx LflywheelMotor, RflywheelMotor, spindexerMotor, hoodMotor; // Shooter mechanics
    private DcMotor intakeMotor; // Intake mechanic
    //private CRServo fspin, rspin, lspin; // Spindex (Carousel) servos
    private Servo llift, rlift; // Lift/feeder servos
    private Limelight3A limelight; // AI Camera
    private colorsensors bench; // Custom color sensor manager
    private ColorSensor intakeSensor; // Used here as a Distance Sensor

    // --- STATE VARIABLES ---
    // These remember what the robot is doing right now.
    private int carouselIndex = 0; // Which slot (0, 1, 2) is active
    private int detectedApriltagId = -1; // ID of the tag we saw

    // --- PHYSICS CONSTANTS ---
    // Constants (final) for the projectile motion calculator.
    final double CAMERA_HEIGHT_INCHES = 16.25;
    final double TAG_HEIGHT_INCHES    = 29.5;
    final double MOUNT_ANGLE_DEGREES  = 10.6; // Camera tilt
    final double SHOOTER_HEIGHT = 17.7;
    final double BASKET_HEIGHT  = 43.0;
    final double ENTRY_ANGLE    = -45; // Desired arc angle
    final double MAX_HOOD_DEGREES = 60.0;
    final double HOOD_TICKS_PER_DEGREE = 3.958;
    final double FLYWHEEL_TICKS_PER_REV = 28.0;
    final double FLYWHEEL_RADIUS = 1.89;
    final double SPEED_SCALAR = 3; // Speed tuner
    private static final double G = 386.1; // Gravity (inches/sec^2)


    // --- CONSTANTS ---
    final double SPINDEXER_KP = 0.003;
    final double SPINDEXER_KF = 0.075;
    final double TICKS_PER_COMPARTMENT = 250.5;

    final double RPM_TOLERANCE = 150.0;


    // --- SAFETY CONSTANT ---
    // Time (ms) to wait for lifts to physically lower before spinning spindexer
    final long LIFT_SAFETY_DELAY_MS = 450;

    // --- ROTATION VARIABLES ---
    private boolean isRotating = false; // Are we currently spinning the carousel?
    private ElapsedTime rotationTimer = new ElapsedTime(); // Timer for the spin

    // UPDATED: Set to 410.0 as requested
    private final double ROTATION_TIME_MS = 410.0;

    private int currentSpindexerSlot = 0;
    private int spindexerTargetPos = 0;

    private boolean liftIsUp = false;

    enum ShootState {
        STOPPED,
        AIMING_AND_SPINUP,
        FIRING_LIFT_UP_1,
        FIRING_RESET_1,
        FIRING_SPIN_INDEXER,
        FIRING_LIFT_UP_2,
        FIRING_RESET_2
    }

    private ShootState shootState = ShootState.STOPPED;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. INIT PHASE (Before Play is pressed)
        initHardware(); // Map all hardware names

        // Setup Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        // Define Starting Position: (122.6, 122.0) facing 36.4 degrees
        follower.setStartingPose(new Pose(122.600, 122.000, Math.toRadians(36.4)));
        paths = new Paths(follower); // Build the paths

        telemetry.addData("Status", "Initialized (CLOSE RED).");
        telemetry.update();

        // Wait here until the driver presses PLAY
        waitForStart();

        // 2. RUN PHASE (After Play is pressed)
        if (opModeIsActive()) {

            // --- SEGMENT 1: DRIVE TO OBSERVE (Path 1) ---
            // Drive to a spot to look at the Obelisk/Wall
            follower.followPath(paths.Path1, true);
            driveWithActions(false); // Wait for drive to finish

            // --- END PATH 1 TASKS ---
            // 1. Scan Obelisk Tag to figure out randomization
            int scannedId = readObeliskTagInstant();
            if (scannedId != -1) {
                detectedApriltagId = scannedId;
                telemetry.addData("OBELISK SCAN", "Found Tag: %d", detectedApriltagId);
            } else {
                telemetry.addData("OBELISK SCAN", "No Tag Found (Defaulting)");
            }
            telemetry.update();

            // 2. Start Shooter Motors (Pre-spin them so they are ready)
            autoAimAndSpinUp();

            // --- SEGMENT 2: PREP TO SHOOT (Path 2) ---
            // Turn to face the goal
            follower.followPath(paths.Path2, true);
            driveWithActions(false);

            // --- END PATH 2 TASKS ---
            // Hold position specifically to ensure accuracy while shooting
            follower.holdPoint(new Pose(90.000, 100.000, Math.toRadians(45)));
            performSmartShot(); // Run the color-sorted shooting logic

            // --- SEGMENT 3: APPROACH SAMPLE 1 (Path 3) ---
            // Drive towards the first yellow sample
            follower.followPath(paths.Path3, true);
            driveWithActions(false);

            // --- END PATH 3 TASKS ---
            // 1. Slow down chassis for accurate intake
            follower.setMaxPower(0.4);

            // --- SEGMENT 4: INTAKE SAMPLE 1 (Path 4) ---
            // Drive THROUGH the sample
            follower.followPath(paths.Path4, true);
            // Run special loop: Drive + Intake Motor + Carousel Logic
            driveWithIntakeAndSpindex();

            // --- END PATH 4 TASKS ---
            // 1. Chassis Full Speed for return trip
            follower.setMaxPower(1.0);
            // 2. Start Shooter Motors (Pre-spin)
            autoAimAndSpinUp();

            // --- SEGMENT 5: RETURN TO SHOOT (Path 5) ---
            // Drive backwards to shooting spot
            follower.followPath(paths.Path5, true);
            driveWithActions(false);

            // --- END PATH 5 TASKS ---
            // Hold and Shoot
            follower.holdPoint(new Pose(90.000, 100.000, Math.toRadians(45)));
            performSmartShot();

            // --- SEGMENT 6: APPROACH SAMPLE 2 (Path 6) ---
            follower.followPath(paths.Path6, true);
            driveWithActions(false);

            // --- END PATH 6 TASKS ---
            follower.setMaxPower(0.4); // Slow down

            // --- SEGMENT 7: INTAKE SAMPLE 2 (Path 7) ---
            follower.followPath(paths.Path7, true);
            driveWithIntakeAndSpindex(); // Run Intake Logic

            // --- END PATH 7 TASKS ---
            follower.setMaxPower(1.0); // Full speed
            autoAimAndSpinUp(); // Pre-spin shooter

            // --- SEGMENT 8: RETURN TO SHOOT 2 (Path 8) ---
            follower.followPath(paths.Path8, true);
            driveWithActions(false);

            // --- END PATH 8 TASKS ---
            // Hold and Shoot final sample
            follower.holdPoint(new Pose(90.000, 100.000, Math.toRadians(45)));
            performSmartShot();

            // DONE: Stop everything
            stopShooter();
        }
    }

    // =========================================================================
    //   DRIVING LOOPS & LOGIC
    // =========================================================================

    private void rotate() {
        // --- 1. SHOOTER CONTROL OVERRIDE ---
        // If the shooter logic is manually spinning the indexer (The "Nudge"),
        // we simply RETURN. We do NOT set power to 0.
        // This lets updateShooterLogic() maintain the 0.375 power.
        if (shootState == ShootState.FIRING_SPIN_INDEXER) {
            return;
        }

        // --- 2. SAFETY GUARD ---
        // Disable PID and FORCE STOP if:
        // A. Lifts are physically up.
        // B. Lifts are currently falling (RESET states), so we don't hit them.
        if (liftIsUp ||
                shootState == ShootState.FIRING_RESET_1 ||
                shootState == ShootState.FIRING_RESET_2) {

            spindexerMotor.setPower(0);
            return;
        }

        // --- 3. NORMAL PID BEHAVIOR ---
        double preciseTarget = currentSpindexerSlot * TICKS_PER_COMPARTMENT;
        spindexerTargetPos = (int) Math.round(preciseTarget);

        int currentPos = spindexerMotor.getCurrentPosition();
        double error = spindexerTargetPos - currentPos;

        if (Math.abs(error) < 5) {
            spindexerMotor.setPower(0);
            return;
        }

        double pTerm = error * SPINDEXER_KP;
        double fTerm = Math.signum(error) * SPINDEXER_KF;
        double power = pTerm + fTerm;
        power = Math.max(-0.6, Math.min(0.6, power));

        spindexerMotor.setPower(power);
    }
    /**
     * Standard drive loop.
     * Keeps the follower updating and telemetry active.
     */
    private void driveWithActions(boolean intakeOn) {
        if(intakeOn) intakeMotor.setPower(0.5); // Turn on intake if requested
        else intakeMotor.setPower(0);

        // While the robot is still moving along the path...
        while (opModeIsActive() && follower.isBusy()) {
            follower.update(); // Calculate physics
            autoAimAndSpinUp(); // Keep flywheels maintaining correct speed

            // Send robot drawing to dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().drawImage("/images/field.png", 0, 0, 144, 144);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        intakeMotor.setPower(0); // Ensure intake is off at end
    }

    /**
     * Intake Drive Loop.
     * Sets Intake to 0.5.
     * Handles Spindex rotation if ball detected.
     */
    private void driveWithIntakeAndSpindex() {
        intakeMotor.setPower(0.5); // Intake ON
        isRotating = false; // Reset flag

        while (opModeIsActive() && follower.isBusy()) {
            follower.update(); // Update Position
            autoAimAndSpinUp(); // Keep flywheels spinning

            // --- SPINDEX LOGIC ---
            if (isRotating) {
                // If currently rotating, check if time is up
                if (rotationTimer.milliseconds() > ROTATION_TIME_MS) {
                    //stopCarousel(); // Stop servos
                    isRotating = false;
                    carouselIndex++; // Advance index (0->1->2)
                    if (carouselIndex > 2) carouselIndex = 0;
                }
            } else {
                // IDLE: Check if we see a ball and have room

                // Check color sensors to see if chambers are full
                boolean chamberFull = (isColor(bench.left, colorsensors.DetectedColor.GREEN) ||
                        isColor(bench.left, colorsensors.DetectedColor.PURPLE)) &&
                        (isColor(bench.right, colorsensors.DetectedColor.GREEN) ||
                                isColor(bench.right, colorsensors.DetectedColor.PURPLE));

                // Check distance sensor (casted from ColorSensor)
                double distanceMM = ((DistanceSensor) intakeSensor).getDistance(DistanceUnit.MM);
                boolean ballAtIntake = distanceMM < 40; // 40mm threshold

                // If ball is there and we aren't full -> Spin!
                if (ballAtIntake && !chamberFull) {
                    //startCarouselRotation();
                    rotate();
                }
                telemetry.addData("Intake Dist", "%.1f mm", distanceMM);
            }
            telemetry.addData("INTAKE", "ACTIVE | Speed: 0.5");
            telemetry.update();
        }

        intakeMotor.setPower(0); // Stop Intake
        //stopCarousel(); // Stop Servos
    }

    // =========================================================================
    //   SMART SHOOTER LOGIC (Same as FarRedAuto)
    // =========================================================================

    private void performSmartShot() {
        // Choose sequence based on ID seen at start
        if (detectedApriltagId == 21) shootGPP();
        else if (detectedApriltagId == 22) shootPGP();
        else if (detectedApriltagId == 23) shootPPG();
        else shootPPG(); // Default
    }

    private void shootGPP() {
        telemetry.addData("Shooting", "GPP Sequence");
        telemetry.update();
        autoAimAndSpinUp(); // Maintain RPM

        // Check chamber status and shoot accordingly
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
            // Manual fallback
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

    // --- ROTATION LOGIC (FIXED) ---
    /*private void rotate() {
        autoAimAndSpinUp();

        // UPDATED: Use the variable (410.0), cast to long for sleep()
        long sleepTime = (long) ROTATION_TIME_MS;

        if (carouselIndex == 2) {
            carouselIndex = 0;
            lspin.setPower(-1);
            sleep(sleepTime); // Uses 410ms
            lspin.setPower(0);
        } else if (carouselIndex == 1) {
            carouselIndex += 1;
            rspin.setPower(-1);
            sleep(sleepTime); // Uses 410ms
            rspin.setPower(0);
        } else if (carouselIndex == 0) {
            carouselIndex += 1;
            fspin.setPower(-1);
            sleep(sleepTime); // Uses 410ms
            fspin.setPower(0);
        }
    }*/

    /*private void startCarouselRotation() {
        isRotating = true;
        rotationTimer.reset();
        if (carouselIndex == 0) fspin.setPower(-1);
        else if (carouselIndex == 1) rspin.setPower(-1);
        else if (carouselIndex == 2) lspin.setPower(-1);
    }*/

    /*private void stopCarousel() {
        fspin.setPower(0); rspin.setPower(0); lspin.setPower(0);
    }*/

    // =========================================================================
    //   VISION & CALCULATIONS
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

    // Physics Calculator for Flywheel & Hood
    private void autoAimAndSpinUp() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy(); // Vertical offset
            double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEGREES + ty);
            double currentDist = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

            double heightY = BASKET_HEIGHT - SHOOTER_HEIGHT;
            ShotData shot = calculateShot(currentDist, heightY, ENTRY_ANGLE);

            if (shot.isPossible) {
                // Set Hood Angle
                double physicsAngle = shot.launchAngleDegrees;
                double invertedAngle = MAX_HOOD_DEGREES - physicsAngle;
                int targetTicks = (int) Math.round(10 * invertedAngle * HOOD_TICKS_PER_DEGREE);
                targetTicks = Math.max(0, Math.min((int)(MAX_HOOD_DEGREES * HOOD_TICKS_PER_DEGREE), targetTicks));
               /* hoodMotor.setTargetPosition(targetTicks);
                hoodMotor.setPower(1.0);*/

                // Set Flywheel Velocity
                double targetRPM = (shot.launchVelocityInchesPerSec * 60) / (2 * Math.PI * FLYWHEEL_RADIUS);
                targetRPM *= SPEED_SCALAR;
                double targetVelocityTicks = (targetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;

                double leftVel = targetVelocityTicks;
                // Spin shot adjustment for long range
                if (currentDist > 120.0) leftVel *= 1.75;

                LflywheelMotor.setVelocity(leftVel);
                RflywheelMotor.setVelocity(targetVelocityTicks);
            }
        }
    }

    // Kinematics Math
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

    // Hardware Mapping
    private void initHardware() {
        LflywheelMotor = hardwareMap.get(DcMotorEx.class, "lgun");
        LflywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 0, 1.5, 16.45));

        RflywheelMotor = hardwareMap.get(DcMotorEx.class, "rgun");
        RflywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 0, 1.5, 16.45));

        /*hoodMotor = hardwareMap.get(DcMotorEx.class, "hood");
        hoodMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hoodMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hoodMotor.setTargetPosition(0);
        hoodMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hoodMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        /*fspin = hardwareMap.crservo.get("fspin");
        rspin = hardwareMap.crservo.get("rspin");
        lspin = hardwareMap.crservo.get("lspin");*/
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
    //   NEW PATH DEFINITIONS (CLOSE RED)
    // =========================================================================

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;

        public Paths(Follower follower) {

            // Path 1: Start (Top Right) -> Shooting Spot
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(122.600, 122.000),
                                    new Pose(90.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36.4), Math.toRadians(36.4))
                    .build();

            // Path 2: Adjust Heading to face goal
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.000, 100.000),
                                    new Pose(90.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(45))
                    .build();

            // Path 3: Shooting Spot -> Align with Sample 1
            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.000, 100.000),
                                    new Pose(104.000, 91.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path 4: Drive THROUGH Sample 1 (Intake ON)
            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(104.000, 91.000),
                                    new Pose(125.000, 91.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path 5: Return to Shoot
            // .setReversed() means the robot drives backwards
            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 91.000),
                                    new Pose(90.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .setReversed()
                    .build();

            // Path 6: Shoot -> Align with Sample 2
            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.000, 100.000),
                                    new Pose(104.000, 66.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path 7: Drive THROUGH Sample 2 (Intake ON)
            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(104.000, 66.000),
                                    new Pose(125.000, 66.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path 8: Return to Shoot final time
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 66.000),
                                    new Pose(90.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();
        }
    }
}