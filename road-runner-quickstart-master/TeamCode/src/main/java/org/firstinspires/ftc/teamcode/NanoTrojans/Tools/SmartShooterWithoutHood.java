package org.firstinspires.ftc.teamcode.NanoTrojans.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Smart Shooter - NO HOOD", group = "Competition")
public class SmartShooterWithoutHood extends LinearOpMode {

    // --- HARDWARE ---
    private Limelight3A limelight;
    private DcMotorEx LflywheelMotor;
    private DcMotorEx RflywheelMotor;
    // Hood motor removed

    // --- CONFIGURATION ---
    // IMPORTANT: Set this to your actual physical hood angle (e.g., 45 degrees)
    final double FIXED_LAUNCH_ANGLE = 45.0;

    // --- MEASUREMENTS (For Distance Calc) ---
    final double CAMERA_HEIGHT_INCHES = 16.25;
    final double TAG_HEIGHT_INCHES    = 29.5;
    final double MOUNT_ANGLE_DEGREES  = 10.6;

    // --- MEASUREMENTS (For Physics Calc) ---
    final double SHOOTER_HEIGHT = 17.7;
    final double BASKET_HEIGHT  = 43.0;

    // --- CONSTANTS ---
    final double FLYWHEEL_TICKS_PER_REV = 28.0;
    final double FLYWHEEL_RADIUS = 1.89;
    final double SPEED_SCALAR = 3;
    private static final double G = 386.1; // Gravity in inches/s^2

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

        PIDFCoefficients lpidfNew = new PIDFCoefficients(100, 0, 1.5, 16.45);
        LflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lpidfNew);

        // 2.2. INIT FLYWHEEL Right
        RflywheelMotor = hardwareMap.get(DcMotorEx.class, "rgun");
        RflywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients rpidfNew = new PIDFCoefficients(100, 0, 1.5, 16.45);
        RflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, rpidfNew);

        // 3. (Hood Init Removed)

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            double currentDist = 0;
            double targetRPM = 0;
            double leftVelocity = 0;
            double rightVelocity = 0;
            boolean shotPossible = false;

            if (result != null && result.isValid()) {
                // --- STEP 1: DISTANCE ---
                double ty = result.getTy();
                double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEGREES + ty);
                currentDist = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

                // --- STEP 2: PHYSICS (FIXED ANGLE) ---
                double heightY = BASKET_HEIGHT - SHOOTER_HEIGHT;

                // Calculate required velocity for our specific Fixed Angle
                double launchVel = calculateFixedAngleVelocity(currentDist, heightY, FIXED_LAUNCH_ANGLE);

                if (launchVel > 0) {
                    shotPossible = true;

                    // --- STEP 3: FLYWHEEL CONTROL ---
                    targetRPM = (launchVel * 60) / (2 * Math.PI * FLYWHEEL_RADIUS);
                    targetRPM *= SPEED_SCALAR;

                    double targetVelocityTicks = (targetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;

                    leftVelocity = targetVelocityTicks;
                    rightVelocity = targetVelocityTicks;

                    // Keep your existing long-range boost logic
                    if (currentDist > 120.0) {
                        leftVelocity *= 1.80;
                    }

                    LflywheelMotor.setVelocity(leftVelocity);
                    RflywheelMotor.setVelocity(rightVelocity);
                } else {
                    // Stop if shot is impossible (too close/too far for angle)
                    LflywheelMotor.setVelocity(0);
                    RflywheelMotor.setVelocity(0);
                }
            }

            // --- TELEMETRY ---
            double LactualRPM = (LflywheelMotor.getVelocity() * 60) / FLYWHEEL_TICKS_PER_REV;
            double RactualRPM = (RflywheelMotor.getVelocity() * 60) / FLYWHEEL_TICKS_PER_REV;

            telemetry.addData("Status", "NO HOOD MODE");
            telemetry.addData("Fixed Angle", "%.1f deg", FIXED_LAUNCH_ANGLE);
            telemetry.addData("1. Distance", "%.2f in", currentDist);
            telemetry.addData("2. Shot Possible?", shotPossible);
            telemetry.addData("3. Left RPM", "Target: %.0f | Actual: %.0f", targetRPM, LactualRPM);
            telemetry.addData("4. Right RPM", "Target: %.0f | Actual: %.0f", targetRPM, RactualRPM);
            telemetry.update();
        }
        limelight.stop();
    }

    /**
     * Calculates the necessary launch velocity to hit the target
     * given a FIXED launch angle.
     */
    private double calculateFixedAngleVelocity(double x, double y, double angleDegrees) {
        double thetaRad = Math.toRadians(angleDegrees);
        double cosTheta = Math.cos(thetaRad);
        double tanTheta = Math.tan(thetaRad);

        // Standard Projectile Motion Formula for Velocity:
        // V = sqrt( (g * x^2) / (2 * cos^2(theta) * (x * tan(theta) - y)) )

        double numerator = G * Math.pow(x, 2);
        double denominator = 2 * Math.pow(cosTheta, 2) * ((x * tanTheta) - y);

        if (denominator <= 0) {
            // Target is unreachable at this angle (usually too high relative to distance)
            return -1;
        } else {
            return Math.sqrt(numerator / denominator);
        }
    }
}