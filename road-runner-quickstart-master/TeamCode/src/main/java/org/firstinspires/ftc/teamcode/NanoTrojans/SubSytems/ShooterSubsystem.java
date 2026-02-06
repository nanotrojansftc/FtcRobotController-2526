package org.firstinspires.ftc.teamcode.NanoTrojans.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem {
    private DcMotorEx LflywheelMotor, RflywheelMotor;
    private Servo hoodServo;
    private Telemetry telemetry;

    // --- CONSTANTS FROM YOUR ORIGINAL CODE ---
    private final double SERVO_LOW_POS = 0.5, SERVO_HIGH_POS = 0.58;
    private final double SERVO_THRESHOLD_DIST = 90.0;
    private final double CAMERA_HEIGHT_INCHES = 16.25, TAG_HEIGHT_INCHES = 29.5;
    private final double MOUNT_ANGLE_DEGREES = 10.6, SHOOTER_HEIGHT = 17.7;
    private final double BASKET_HEIGHT = 43.0, ENTRY_ANGLE = -45.0, G = 386.1;
    private final double FLYWHEEL_TICKS_PER_REV = 28.0, FLYWHEEL_RADIUS = 1.89, SPEED_SCALAR = 2.8;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        LflywheelMotor = hardwareMap.get(DcMotorEx.class, "lgun");
        RflywheelMotor = hardwareMap.get(DcMotorEx.class, "rgun");
        hoodServo = hardwareMap.get(Servo.class, "hood");

        LflywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RflywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients tunedPIDF = new PIDFCoefficients(70, 0, 1.5, 13.5);
        LflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tunedPIDF);
        RflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tunedPIDF);

        LflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * The "Brain" of the subsystem. Call this in your loop with the latest Limelight data.
     */
    public void update(LLResult result) {
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEGREES + ty);
            double currentDist = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

            ShotData shot = calculateShot(currentDist, BASKET_HEIGHT - SHOOTER_HEIGHT, ENTRY_ANGLE);

            if (shot.isPossible) {
                double targetRPM = (shot.launchVelocityInchesPerSec * 60) / (2 * Math.PI * FLYWHEEL_RADIUS) * SPEED_SCALAR;
                double ticks = (targetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;

                hoodServo.setPosition(currentDist < SERVO_THRESHOLD_DIST ? SERVO_LOW_POS : SERVO_HIGH_POS);

                // Distance compensation from your code
                double leftTicks = (currentDist > 120.0) ? ticks * 1.80 : ticks;

                LflywheelMotor.setVelocity(leftTicks);
                RflywheelMotor.setVelocity(ticks);

                telemetry.addData("Shooter", "LOCKED - Dist: %.1f", currentDist);
                telemetry.addData("Target RPM", "%.0f", targetRPM);
            }
        } else {
            stop(); // Or maintain state depending on preference
        }
    }

    public void stop() {
        LflywheelMotor.setVelocity(0);
        RflywheelMotor.setVelocity(0);
        hoodServo.setPosition(SERVO_LOW_POS);
    }

    private ShotData calculateShot(double x, double y, double theta) {
        ShotData data = new ShotData();
        double thetaRad = Math.toRadians(theta);
        double alphaRad = Math.atan(((2 * y) / x) - Math.tan(thetaRad));
        double denominator = 2 * Math.pow(Math.cos(alphaRad), 2) * ((x * Math.tan(alphaRad)) - y);

        if (denominator <= 0) {
            data.isPossible = false;
        } else {
            data.isPossible = true;
            data.launchVelocityInchesPerSec = Math.sqrt((G * Math.pow(x, 2)) / denominator);
        }
        return data;
    }

    private static class ShotData {
        public double launchVelocityInchesPerSec;
        public boolean isPossible;
    }
}