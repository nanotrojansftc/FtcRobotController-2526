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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;
import java.util.List;

@Autonomous(name = "Far Red Auto", group = "Competition")
public class FarRedAuto extends LinearOpMode {

    private Follower follower;
    private Paths paths;
    private DcMotorEx LflywheelMotor, RflywheelMotor;

    // CHANGED: Replaced hoodMotor with hoodServo
    private Servo hoodServo;

    private DcMotorEx spindexerMotor;
    private DcMotor intakeMotor;
    private Servo llift, rlift;
    private Limelight3A limelight;
    private colorsensors bench;
    private ColorSensor intakeSensor;

    private int detectedApriltagId = -1;

    // --- TUNED SPINDEXER CONSTANTS ---
    final double SPINDEXER_KP = 0.003;
    final double SPINDEXER_KF = 0.075;
    final double TICKS_PER_COMPARTMENT = 250.5;

    // Spindexer State Tracking
    private int currentSpindexerSlot = 0;
    private int spindexerTargetPos = 0;
    private ElapsedTime spindexerCooldown = new ElapsedTime();

    // --- SHOOTER VARIABLES ---
    private double targetLeftVel = 0;
    private double targetRightVel = 0;
    private final double RPM_TOLERANCE = 150.0;

    // --- HOOD SERVO CALIBRATION (MUST TUNE THESE) ---
    // See PDF Page 7: "Min angle a1 = Servo position s1" [cite: 137]
    // Move servo to a low position. Record the Position (0.0-1.0) and the physical Angle (degrees).
    final double SERVO_POS_1 = 0.2;  // <--- UPDATE THIS
    final double ANGLE_AT_POS_1 = 10.0; // <--- UPDATE THIS (Physical degrees)

    // Move servo to a high position. Record the Position and Angle.
    final double SERVO_POS_2 = 0.8;  // <--- UPDATE THIS
    final double ANGLE_AT_POS_2 = 55.0; // <--- UPDATE THIS (Physical degrees)

    // Field & Robot Constants
    final double CAMERA_HEIGHT_INCHES = 16.25;
    final double TAG_HEIGHT_INCHES    = 29.5;
    final double MOUNT_ANGLE_DEGREES  = 10.6;
    final double SHOOTER_HEIGHT = 17.7;
    final double BASKET_HEIGHT  = 43.0;
    final double ENTRY_ANGLE    = -45;

    // Kept for limit checks, though servo mapping handles the main logic
    final double MAX_HOOD_DEGREES = 60.0;
    final double FLYWHEEL_TICKS_PER_REV = 28.0;
    final double FLYWHEEL_RADIUS = 1.89;
    final double SPEED_SCALAR = 3;
    private static final double G = 386.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(87.500, 8.000, Math.toRadians(90)));
        paths = new Paths(follower);

        telemetry.addData("Status", "Initialized. Hood Servo Mode Active.");
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
            driveWithActions(false); // Drive to shoot pos
            follower.holdPoint(new Pose(87.5, 27.25, Math.toRadians(64)));
            performSmartShot();

            // --- SEGMENT 2 (Drive to Samples) ---
            follower.followPath(paths.Path2, true);
            driveWithActions(false);

            // --- SEGMENT 3 (Intake First Sample) ---
            follower.setMaxPower(0.5);
            follower.followPath(paths.Path3, true);
            driveWithIntakeAndSpindex();
            follower.setMaxPower(1.0);

            // --- SEGMENT 4 (Drive Back) ---
            follower.followPath(paths.Path4, true);
            driveWithActions(false);
            follower.holdPoint(new Pose(86.75, 27.5, Math.toRadians(64)));
            performSmartShot();

            // --- SEGMENT 5 ---
            follower.followPath(paths.Path5, true);
            driveWithActions(false);

            // --- SEGMENT 6 (Intake Second Sample) ---
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
    //   SPINDEXER LOGIC (Unchanged)
    // =========================================================================

    private void updateSpindexer(boolean autoIndexEnabled) {
        // 1. AUTO-INDEXING LOGIC
        if (autoIndexEnabled && spindexerCooldown.milliseconds() > 1000) {
            boolean hasSample = bench.detectByHue(bench.back, telemetry) != colorsensors.DetectedColor.UNKNOWN;
            if (hasSample) {
                currentSpindexerSlot++;
                spindexerCooldown.reset();
            }
        }

        // 2. TARGET CALCULATION
        double preciseTarget = currentSpindexerSlot * TICKS_PER_COMPARTMENT;
        spindexerTargetPos = (int) Math.round(preciseTarget);

        // 3. PIDF CONTROL
        int currentPos = spindexerMotor.getCurrentPosition();
        double error = spindexerTargetPos - currentPos;
        double pTerm = error * SPINDEXER_KP;
        double fTerm = 0;
        if (Math.abs(error) > 5) {
            fTerm = Math.signum(error) * SPINDEXER_KF;
        }
        double power = pTerm + fTerm;
        power = Math.max(-1.0, Math.min(1.0, power));
        spindexerMotor.setPower(power);
    }

    private void rotate() {
        currentSpindexerSlot++;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < 400) {
            updateSpindexer(false);
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
            updateSpindexer(false);

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().drawImage("/images/field.png", 0, 0, 144, 144);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        intakeMotor.setPower(0);
    }

    private void driveWithIntakeAndSpindex() {
        intakeMotor.setPower(0.75);
        if (spindexerCooldown.seconds() > 2) spindexerCooldown.reset();

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            autoAimAndSpinUp();
            updateSpindexer(true); // Allow auto-index

            telemetry.addData("INTAKE", "Scanning for samples...");
            telemetry.addData("Spindexer Slot", currentSpindexerSlot);
            telemetry.update();
        }
        intakeMotor.setPower(0);
    }

    // =========================================================================
    //   SHOOTING LOGIC
    // =========================================================================

    private void performSmartShot() {
        if (detectedApriltagId == 21) shootGPP();
        else if (detectedApriltagId == 22) shootPGP();
        else if (detectedApriltagId == 23) shootPPG();
        else shootPPG();
    }

    private void shootGPP() {
        waitForFlywheelReady();
        if (isColor(bench.left, colorsensors.DetectedColor.GREEN) &&
                isColor(bench.right, colorsensors.DetectedColor.PURPLE)) {
            safeShootDual(true, true);
            smartSleep(250);
            rotatePurpleIn();
            safeShootPurple();
        } else {
            rotateGreenIn();
            if (!safeShootGreen()) { rotateGreenIn(); safeShootGreen(); }
            smartSleep(250);
            if (!safeShootPurple()) { rotatePurpleIn(); safeShootPurple(); }
            smartSleep(250);
            if (!safeShootPurple()) { rotatePurpleIn(); safeShootPurple(); }
        }
    }

    private void shootPGP() {
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
    //   HELPER ACTIONS
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
        waitForFlywheelReady();
        if (isGPP) {
            llift.setPosition(0.625);
            smartSleep(200);
            waitForFlywheelReady();
            rlift.setPosition(0.3);
        } else {
            rlift.setPosition(0.3);
            smartSleep(200);
            waitForFlywheelReady();
            llift.setPosition(0.625);
        }
        smartSleep(300);
        llift.setPosition(1); rlift.setPosition(0.005);
    }

    private void performSingleShotLeft() {
        waitForFlywheelReady();
        llift.setPosition(0.625);
        smartSleep(300);
        llift.setPosition(1);
    }

    private void performSingleShotRight() {
        waitForFlywheelReady();
        rlift.setPosition(0.3);
        smartSleep(300);
        rlift.setPosition(0.005);
    }

    private boolean rotateGreenIn() {
        rotate();
        if (!greenInChamber()) { rotate(); }
        if (!greenInChamber()) { rotate(); }
        return greenInChamber();
    }

    private boolean rotatePurpleIn() {
        rotate();
        if (!purpleInChamber()) { rotate(); }
        if (!purpleInChamber()) { rotate(); }
        return purpleInChamber();
    }

    public void smartSleep(long milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            follower.update();
            autoAimAndSpinUp();
            updateSpindexer(false);
        }
    }

    private void waitForFlywheelReady() {
        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && timeout.seconds() < 1.0) {
            follower.update();
            autoAimAndSpinUp();
            updateSpindexer(false);

            double currentLeft = LflywheelMotor.getVelocity();
            double currentRight = RflywheelMotor.getVelocity();
            boolean leftReady = Math.abs(currentLeft - targetLeftVel) < RPM_TOLERANCE;
            boolean rightReady = Math.abs(currentRight - targetRightVel) < RPM_TOLERANCE;
            if (leftReady && rightReady) break;
        }
    }

    // =========================================================================
    //   INIT & HARDWARE
    // =========================================================================

    private void initHardware() {
        LflywheelMotor = hardwareMap.get(DcMotorEx.class, "lgun");
        LflywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 0, 1.5, 16.45));

        RflywheelMotor = hardwareMap.get(DcMotorEx.class, "rgun");
        RflywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 0, 1.5, 16.45));

        // --- NEW HOOD SERVO INIT ---
        hoodServo = hardwareMap.get(Servo.class, "hood");
        // If the servo moves opposite to your angle, uncomment this:
        // hoodServo.setDirection(Servo.Direction.REVERSE);
        // Initialize to a safe position (e.g., minimum angle)
        hoodServo.setPosition(SERVO_POS_1);

        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        llift = hardwareMap.get(Servo.class, "llift");
        rlift = hardwareMap.get(Servo.class, "rlift");

        bench = new colorsensors();
        bench.init(hardwareMap);
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

    // =========================================================================
    //   AUTO AIM (UPDATED WITH SERVO MAPPING)
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
                // 1. Get Target Physics Angle
                double physicsAngle = shot.launchAngleDegrees;
                double invertedAngle = MAX_HOOD_DEGREES - physicsAngle; // Kept your original logic

                // 2. Clamp Angle to Calibration Range
                double targetAngle = Math.max(ANGLE_AT_POS_1, Math.min(ANGLE_AT_POS_2, invertedAngle));

                // 3. Map Angle to Servo Position (PDF Page 7 Formula)
                // Position = (s1 - s2)/(a1 - a2) * (angle - a1) + s1
                double slope = (SERVO_POS_1 - SERVO_POS_2) / (ANGLE_AT_POS_1 - ANGLE_AT_POS_2);
                double targetServoPos = slope * (targetAngle - ANGLE_AT_POS_1) + SERVO_POS_1;

                // 4. Set Servo
                hoodServo.setPosition(targetServoPos);

                // 5. Spin Up Flywheels (Original Logic)
                double targetRPM = (shot.launchVelocityInchesPerSec * 60) / (2 * Math.PI * FLYWHEEL_RADIUS);
                targetRPM *= SPEED_SCALAR;
                double targetVelocityTicks = (targetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;

                double leftVel = targetVelocityTicks;
                if (currentDist > 120.0) leftVel *= 1.75;

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