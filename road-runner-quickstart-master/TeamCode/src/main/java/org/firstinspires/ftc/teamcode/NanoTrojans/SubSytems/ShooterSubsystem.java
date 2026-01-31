package org.firstinspires.ftc.teamcode.NanoTrojans.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;

public class ShooterSubsystem {
    // Hardware
    private DcMotorEx lFlywheel, rFlywheel, spindexer;
    private Servo llift, rlift, hood;
    private Limelight3A limelight;
    private colorsensors bench;

    // State Machine
    public enum ShootState { IDLE, PREP_SORT, SPIN_UP, FIRING, ROTATING_NEXT, FINISHED }
    private ShootState currentState = ShootState.IDLE;

    // Physics & PID Constants from Victor's Code
    private final double TICKS_PER_COMPARTMENT = 250.5;
    private final double RPM_TOLERANCE = 150.0;
    private final double FLYWHEEL_RADIUS = 1.89;
    private final double FLYWHEEL_TICKS_PER_REV = 28.0;
    private final double SPEED_SCALAR = 3.0;
    private static final double G = 386.1; // Gravity in inches/sec^2

    // Operational Variables
    private double targetVel = 0;
    private int obeliskTarget = -1;
    private int currentSpindexerSlot = 0;
    private ElapsedTime stateTimer = new ElapsedTime();
    private int shotsFired = 0;

    public ShooterSubsystem(HardwareMap hwMap) {
        lFlywheel = hwMap.get(DcMotorEx.class, "lgun");
        rFlywheel = hwMap.get(DcMotorEx.class, "rgun");
        spindexer = hwMap.get(DcMotorEx.class, "spindexer");
        llift = hwMap.get(Servo.class, "llift");
        rlift = hwMap.get(Servo.class, "rlift");
        hood = hwMap.get(Servo.class, "hood");

        limelight = hwMap.get(Limelight3A.class, "limelight");
        bench = new colorsensors();
        bench.init(hwMap);
    }

    public void update(int obeliskId) {
        this.obeliskTarget = obeliskId;

        switch (currentState) {
            case IDLE:
                stopAll();
                break;

            case PREP_SORT:
                if (alignChambersForGoal()) {
                    stateTimer.reset();
                    currentState = ShootState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                autoAimAndPrep();
                // Settling time check from FarRedAuto logic
                if (isFlywheelReady() && stateTimer.milliseconds() > 300) {
                    stateTimer.reset();
                    currentState = ShootState.FIRING;
                }
                break;

            case FIRING:
                // Command Lifts Up
                llift.setPosition(0.625);
                rlift.setPosition(0.3);

                // Victor's timing for launch
                if (stateTimer.milliseconds() > 400) {
                    shotsFired += 2;
                    stateTimer.reset();
                    currentState = ShootState.ROTATING_NEXT;
                }
                break;

            case ROTATING_NEXT:
                // Command Lifts Down
                llift.setPosition(1.0);
                rlift.setPosition(0.005);

                if (shotsFired >= 3) {
                    currentState = ShootState.FINISHED;
                } else if (stateTimer.milliseconds() > 500) {
                    // Index next slot
                    currentSpindexerSlot++;
                    double targetPos = currentSpindexerSlot * TICKS_PER_COMPARTMENT;

                    // Controlled Spindexer Power
                    double error = targetPos - spindexer.getCurrentPosition();
                    if (Math.abs(error) < 15) {
                        spindexer.setPower(0);
                        stateTimer.reset();
                        currentState = ShootState.SPIN_UP;
                    } else {
                        spindexer.setPower(0.4 * Math.signum(error));
                    }
                }
                break;

            case FINISHED:
                stopAll();
                currentState = ShootState.IDLE;
                break;
        }
    }

    private boolean alignChambersForGoal() {
        colorsensors.DetectedColor left = bench.detectByHue(bench.left, null);
        colorsensors.DetectedColor right = bench.detectByHue(bench.right, null);
        boolean isMatch = false;

        // Sorting Logic based on Obelisk Tags 21-23
        switch (obeliskTarget) {
            case 21: // GPP
                isMatch = (left == colorsensors.DetectedColor.GREEN || right == colorsensors.DetectedColor.GREEN);
                break;
            case 22: // PGP
                isMatch = (left == colorsensors.DetectedColor.PURPLE && right == colorsensors.DetectedColor.GREEN);
                break;
            case 23: // PPG
                isMatch = (left == colorsensors.DetectedColor.PURPLE && right == colorsensors.DetectedColor.PURPLE);
                break;
            default: return true;
        }

        if (isMatch) {
            spindexer.setPower(0);
            return true;
        }
        spindexer.setPower(0.3);
        return false;
    }

    private void autoAimAndPrep() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToGoalRadians = Math.toRadians(10.6 + ty); // MOUNT_ANGLE
            double currentDist = (29.5 - 16.25) / Math.tan(angleToGoalRadians); // Distance formula

            // Servo threshold
            hood.setPosition(currentDist < 90.0 ? 0.5 : 0.58);

            // Physics Trajectory
            double x = currentDist;
            double y = 43.0 - 17.7; // Basket - Shooter Height
            double thetaRad = Math.toRadians(-45); // Entry Angle

            double alphaRad = Math.atan((2 * y / x) - Math.tan(thetaRad));
            double cosAlpha = Math.cos(alphaRad);
            double tanAlpha = Math.tan(alphaRad);

            double denominator = 2 * Math.pow(cosAlpha, 2) * ((x * tanAlpha) - y);

            if (denominator > 0) {
                double launchVel = Math.sqrt((G * Math.pow(x, 2)) / denominator);
                double targetRPM = (launchVel * 60) / (2 * Math.PI * FLYWHEEL_RADIUS) * SPEED_SCALAR;
                targetVel = (targetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;

                lFlywheel.setVelocity(targetVel);
                rFlywheel.setVelocity(targetVel);
            }
        }
    }

    private boolean isFlywheelReady() {
        return Math.abs(lFlywheel.getVelocity() - targetVel) < RPM_TOLERANCE;
    }

    public void startSequence() {
        shotsFired = 0;
        currentState = ShootState.PREP_SORT;
    }

    public void stopAll() {
        lFlywheel.setVelocity(0);
        rFlywheel.setVelocity(0);
        spindexer.setPower(0);
        llift.setPosition(1.0);
        rlift.setPosition(0.005);
    }
}