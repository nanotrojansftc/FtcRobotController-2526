package org.firstinspires.ftc.teamcode.NanoTrojans.SubSytems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem {

    // --- HARDWARE ---
    private DcMotorEx LflywheelMotor, RflywheelMotor, spindexerMotor;
    private Servo hoodServo, llift, rlift;
    private Telemetry telemetry;

    // --- MEMORY VARIABLES (for Limelight shooting) ---
    private ElapsedTime signalLossTimer = new ElapsedTime();
    private boolean hasTargetMemory = false;
    private double lastTargetLeftTicks = 0;
    private double lastTargetRightTicks = 0;
    private double lastServoPos = 0.5;
    private double currentDist = 0;

    // --- CONSTANTS ---
    private final double FLICKER_THRESHOLD_MS = 1000;
    private final double SERVO_LOW_POS = 0.5, SERVO_HIGH_POS = 0.58;
    private final double SERVO_THRESHOLD_DIST = 80.0;

    // --- LIFT CONSTANTS (From TeleOpAutomation) ---
    private final double LEFT_LIFT_UP = 0.625;
    private final double LEFT_LIFT_DOWN = 1.0;
    private final double RIGHT_LIFT_UP = 0.3;
    private final double RIGHT_LIFT_DOWN = 0.005;

    // --- PHYSICS CONSTANTS ---
    private final double CAMERA_HEIGHT_INCHES = 16.25, TAG_HEIGHT_INCHES = 29.5;
    private final double MOUNT_ANGLE_DEGREES = 10.6, SHOOTER_HEIGHT = 17.7;
    private final double BASKET_HEIGHT = 43.0, ENTRY_ANGLE = -45.0, G = 386.1;
    private final double FLYWHEEL_TICKS_PER_REV = 28.0, FLYWHEEL_RADIUS = 1.89, SPEED_SCALAR = 2.8;

    // --- SPINDEXER PID (same style as TeleOpAutomation) ---
    private final double SPINDEXER_KP = 0.003;
    private final double SPINDEXER_KF = 0.075;
    private final double TICKS_PER_COMPARTMENT = 250.5;   // tune this on-bot

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        LflywheelMotor = hardwareMap.get(DcMotorEx.class, "lgun");
        RflywheelMotor = hardwareMap.get(DcMotorEx.class, "rgun");
        hoodServo      = hardwareMap.get(Servo.class, "hood");
        llift          = hardwareMap.get(Servo.class, "llift");
        rlift          = hardwareMap.get(Servo.class, "rlift");
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexer");

        // Spindexer basic setup
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetLifts(); // Force down on init

        LflywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RflywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients tunedPIDF = new PIDFCoefficients(70, 0, 1.5, 13.5);
        LflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tunedPIDF);
        RflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tunedPIDF);

        LflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        signalLossTimer.reset();
    }

    // =========================================================
    // PART 1: LIMELIGHT + FLYWHEEL CONTROL
    // =========================================================

    /** Call every loop with latest Limelight result to aim and spin shooter. */
    public void updateLimelightOnly(LLResult result) {
        if (result != null && result.isValid()) {
            signalLossTimer.reset();
            hasTargetMemory = true;

            double ty = result.getTy();
            double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEGREES + ty);
            double currentDist = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

            ShotData shot = calculateShot(currentDist, BASKET_HEIGHT - SHOOTER_HEIGHT, ENTRY_ANGLE);

            if (shot.isPossible) {
                double targetRPM = (shot.launchVelocityInchesPerSec * 60) /
                        (2 * Math.PI * FLYWHEEL_RADIUS) * SPEED_SCALAR;
                double ticks = (targetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;

                lastServoPos = (currentDist < SERVO_THRESHOLD_DIST) ? SERVO_LOW_POS : SERVO_HIGH_POS;

                double leftTicks = (currentDist > 120.0) ? ticks * 1.80 : ticks;

                lastTargetLeftTicks  = leftTicks;
                lastTargetRightTicks = ticks;

                hoodServo.setPosition(lastServoPos);
                LflywheelMotor.setVelocity(lastTargetLeftTicks);
                RflywheelMotor.setVelocity(lastTargetRightTicks);

                telemetry.addData("Shooter", "LOCKED - Dist: %.1f", currentDist);
            }
        } else {
            if (hasTargetMemory && signalLossTimer.milliseconds() < FLICKER_THRESHOLD_MS) {
                hoodServo.setPosition(lastServoPos);
                LflywheelMotor.setVelocity(lastTargetLeftTicks);
                RflywheelMotor.setVelocity(lastTargetRightTicks);

                telemetry.addData("Shooter",
                        "MEMORY HOLD (%.0f ms)",
                        (FLICKER_THRESHOLD_MS - signalLossTimer.milliseconds()));
            } else {
                idle();
                telemetry.addData("Shooter", "IDLE / SEARCHING");
            }
        }
    }

    /** Convenience alias matching older TeleOp code. */
    public void update(LLResult result) {
        updateLimelightOnly(result);
    }

    public boolean isFlywheelReady(double tolerancePercent) {
        if (lastTargetLeftTicks == 0) return false;
        double currentVel = LflywheelMotor.getVelocity();
        double error = Math.abs(currentVel - lastTargetLeftTicks);
        return error < (lastTargetLeftTicks * tolerancePercent);
    }

    // =========================================================
    // PART 2: BLOCKING ACTIONS FOR AUTONOMOUS
    // =========================================================

    /** Blocking: shoot the two back artifacts (both lifts up, wait, down). */
    public void shootBackPair(LinearOpMode opMode) {
        ElapsedTime t = new ElapsedTime();
        moveUpLifts();
        t.reset();
        while (opMode.opModeIsActive() && t.milliseconds() < 300) {
            // just wait for shots to clear
        }
        resetLifts();
        t.reset();
        while (opMode.opModeIsActive() && t.milliseconds() < 400) {
            // allow lifts to fully come down
        }
    }

    /** Blocking: index the carousel exactly one compartment. */
    public void indexOneCompartment(LinearOpMode opMode) {
        // Treat current encoder position as slot 0, move to +TICKS_PER_COMPARTMENT
        int startPos = spindexerMotor.getCurrentPosition();
        double targetPos = startPos + TICKS_PER_COMPARTMENT;

        while (opMode.opModeIsActive()) {
            int currentPos = spindexerMotor.getCurrentPosition();
            double error = targetPos - currentPos;

            if (Math.abs(error) < 10) {
                spindexerMotor.setPower(0);
                break;
            }

            double pTerm = error * SPINDEXER_KP;
            double fTerm = Math.signum(error) * SPINDEXER_KF;
            double power = pTerm + fTerm;
            power = Math.max(-0.6, Math.min(0.6, power));

            spindexerMotor.setPower(power);
        }
    }

    /** Blocking: shoot a single artifact (both lifts up, wait, down). */
    public void shootSingle(LinearOpMode opMode) {
        ElapsedTime t = new ElapsedTime();
        moveUpLifts();
        t.reset();
        while (opMode.opModeIsActive() && t.milliseconds() < 400) {
            // wait for shot to clear
        }
        resetLifts();
        t.reset();
        while (opMode.opModeIsActive() && t.milliseconds() < 400) {
            // allow lifts to fully come down
        }
    }

    // =========================================================
    // GENERAL HELPERS
    // =========================================================

    public void idle() {
        LflywheelMotor.setVelocity(0);
        RflywheelMotor.setVelocity(0);
        hoodServo.setPosition(SERVO_LOW_POS);
        hasTargetMemory = false;
    }

    public void stop() {
        idle();
        resetLifts();
        spindexerMotor.setPower(0);
    }

    public void printDiagnostics(int detectedTagId) {
        telemetry.addData("--- TARGETING ---", "");
        telemetry.addData("Scan ID", detectedTagId);
        telemetry.addData("Distance", "%.1f in", currentDist);

        telemetry.addData("--- RPM DATA ---", "");
        // Convert ticks/sec back to RPM for display if desired, or just show ticks
        telemetry.addData("L Gun", "Tgt: %.0f | Act: %.0f", lastTargetLeftTicks, LflywheelMotor.getVelocity());
        telemetry.addData("R Gun", "Tgt: %.0f | Act: %.0f", lastTargetRightTicks, RflywheelMotor.getVelocity());

        telemetry.addData("Ready?", isFlywheelReady(0.05) ? "YES" : "NO");
    }

    // --- LIFT CONTROLS ---
    public void moveUpLifts() {
        llift.setPosition(LEFT_LIFT_UP);
        rlift.setPosition(RIGHT_LIFT_UP);
    }

    public void resetLifts() {
        llift.setPosition(LEFT_LIFT_DOWN);
        rlift.setPosition(RIGHT_LIFT_DOWN);
    }

    public void moveUpRightLiftOnly() {
        rlift.setPosition(RIGHT_LIFT_UP);
    }

    public void feedShot() {
        moveUpLifts();
    }

    // --- SHOT PHYSICS ---
    private ShotData calculateShot(double x, double y, double theta) {
        ShotData data = new ShotData();
        double thetaRad = Math.toRadians(theta);
        double alphaRad = Math.atan(((2 * y) / x) - Math.tan(thetaRad));
        double denominator = 2 * Math.pow(Math.cos(alphaRad), 2) * ((x * Math.tan(alphaRad)) - y);

        if (denominator <= 0) {
            data.isPossible = false;
        } else {
            data.isPossible = true;
            data.launchVelocityInchesPerSec =
                    Math.sqrt((G * Math.pow(x, 2)) / denominator);
        }
        return data;
    }

    private static class ShotData {
        public double launchVelocityInchesPerSec;
        public boolean isPossible;
    }
}
