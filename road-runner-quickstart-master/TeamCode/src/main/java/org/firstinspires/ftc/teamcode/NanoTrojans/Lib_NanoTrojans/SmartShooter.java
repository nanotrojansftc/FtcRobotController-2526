package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Smart Shooter v2 - Fixed Angle", group = "Competition")
public class SmartShooter extends LinearOpMode {

    // --- HARDWARE ---
    private Limelight3A limelight;
    private DcMotorEx LflywheelMotor;

    private DcMotorEx RflywheelMotor;
    private DcMotorEx hoodMotor;

    // --- MEASUREMENTS (For Distance Calc) ---
    final double CAMERA_HEIGHT_INCHES = 16.25;
    final double TAG_HEIGHT_INCHES    = 29.5;
    final double MOUNT_ANGLE_DEGREES  = 10.6;

    // --- MEASUREMENTS (For Physics Calc) ---
    final double SHOOTER_HEIGHT = 17.7;
    final double BASKET_HEIGHT  = 43.0;

    // CHANGED: Flatter entry angle (-25 instead of -45) to lower the hood
    final double ENTRY_ANGLE    = -45;

    // --- MOTOR CONSTANTS ---
    // 117 RPM Yellow Jacket is ~3.96 ticks per degree
    final double HOOD_TICKS_PER_DEGREE = 3.958;
    final double MAX_HOOD_DEGREES = 60.0;

    // Flywheel Constants
    final double FLYWHEEL_TICKS_PER_REV = 28.0;
    final double FLYWHEEL_RADIUS = 1.89;
    final double SPEED_SCALAR = 3; // Adjusted to a more realistic 25% boost

    @Override
    public void runOpMode() {
        // 1. INIT LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // 2.1. INIT FLYWHEEL left
        LflywheelMotor = hardwareMap.get(DcMotorEx.class, "lgun");
        LflywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Using your proven P=100 and F=16.45
        PIDFCoefficients lpidfNew = new PIDFCoefficients(100, 0, 1.5, 16.45);
        LflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lpidfNew);

        // 2.2. INIT FLYWHEEL Right
        RflywheelMotor = hardwareMap.get(DcMotorEx.class, "rgun");
        RflywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Using your proven P=100 and F=16.45
        PIDFCoefficients rpidfNew = new PIDFCoefficients(100, 0, 1.5, 16.45);
        RflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, rpidfNew);

        // 3. INIT HOOD
        hoodMotor = hardwareMap.get(DcMotorEx.class, "hood");
        hoodMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hoodMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hoodMotor.setTargetPosition(0);
        hoodMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hoodMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            double currentDist = 0;
            double physicsAngle = 0;
            double targetRPM = 0;
            double leftVelocity = 0;
            double rightVelocity = 0;


            if (result != null && result.isValid()) {
                // --- STEP 1: DISTANCE ---
                double ty = result.getTy();
                double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEGREES + ty);
                currentDist = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

                // --- STEP 2: PHYSICS ---
                double heightY = BASKET_HEIGHT - SHOOTER_HEIGHT;
                ShotData shot = calculateShot(currentDist, heightY, ENTRY_ANGLE);

                if (shot.isPossible) {
                    physicsAngle = shot.launchAngleDegrees;

                    // --- STEP 3: HOOD CONTROL (INVERTED LOGIC) ---
                    // Physics 0 deg (Horizontal) -> Should be Max Ticks (Out)
                    // Physics 60 deg (Vertical) -> Should be 0 Ticks (In)
                    double invertedAngle = MAX_HOOD_DEGREES - physicsAngle;

                    int targetTicks = (int) Math.round(10*invertedAngle * HOOD_TICKS_PER_DEGREE);

                    // Clamp between 0 and your physical limit (approx 237 ticks)
                    int safeLimitTicks = (int)(MAX_HOOD_DEGREES * HOOD_TICKS_PER_DEGREE);
                    targetTicks = Math.max(0, Math.min(safeLimitTicks, targetTicks));

                    hoodMotor.setTargetPosition(targetTicks);
                    hoodMotor.setPower(1.0);

                    // --- STEP 4: FLYWHEEL CONTROL ---
                    // --- STEP 4: FLYWHEEL CONTROL ---
                    targetRPM = (shot.launchVelocityInchesPerSec * 60) / (2 * Math.PI * FLYWHEEL_RADIUS);
                    targetRPM *= SPEED_SCALAR;

                    double targetVelocityTicks = (targetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;

                    // Base velocity for both motors
                    leftVelocity = targetVelocityTicks;
                    rightVelocity = targetVelocityTicks;

                    // If we are over 120 inches, boost the left motor by 15%
                    if (currentDist > 120.0) {
                        leftVelocity *= 1.80;
                    }

                    LflywheelMotor.setVelocity(leftVelocity);
                    RflywheelMotor.setVelocity(rightVelocity);
                }
            }



            // --- TELEMETRY ---
            double LactualRPM = (LflywheelMotor.getVelocity() * 60) / FLYWHEEL_TICKS_PER_REV;
            double RactualRPM = (RflywheelMotor.getVelocity() * 60) / FLYWHEEL_TICKS_PER_REV;

            // Capture the actual velocity in Ticks per Second
            double leftActualTicks = LflywheelMotor.getVelocity();
            double rightActualTicks = RflywheelMotor.getVelocity();

            telemetry.addData("1. Distance", "%.2f in", currentDist);
            telemetry.addData("2. Physics Angle", "%.2f deg", physicsAngle);
            telemetry.addData("3. Hood Ticks", "Target: %d | Actual: %d",
                    hoodMotor.getTargetPosition(), hoodMotor.getCurrentPosition());
            telemetry.addData("4. Left RPM", "Target: %.0f | Actual: %.0f", targetRPM, LactualRPM);
            telemetry.addData("4. Right RPM", "Target: %.0f | Actual: %.0f", targetRPM, RactualRPM);
            telemetry.addData("5. Left Ticks", "Target: %.0f | Actual: %.0f", leftVelocity, leftActualTicks);
            telemetry.addData("5. Right Ticks", "Target: %.0f | Actual: %.0f", rightVelocity, rightActualTicks);
            telemetry.update();
        }
        limelight.stop();
    }

    private ShotData calculateShot(double x, double y, double theta) {
        ShotData data = new ShotData();
        double thetaRad = Math.toRadians(theta);

        double term1 = (2 * y) / x;
        double term2 = Math.tan(thetaRad);

        // This calculates the necessary launch angle (alpha)
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