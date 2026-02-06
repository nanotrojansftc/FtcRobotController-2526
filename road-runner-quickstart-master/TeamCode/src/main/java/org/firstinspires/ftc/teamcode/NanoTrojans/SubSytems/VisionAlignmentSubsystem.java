package org.firstinspires.ftc.teamcode.NanoTrojans.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class VisionAlignmentSubsystem {

    // --------------------- PD controller ----------------------
    private double kP = 0.019;
    private double kD = 0.0001;
    private double goalX = 0.0;
    private double angleTolerance = 0.4;
    private double error = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private double lastRotateCmd = 0.0;

    private final Limelight3A limelight;
    private final Follower follower;

    // -------------------- Smart Shooter Constants -----------------------
    private static final double CAMERA_HEIGHT_INCHES = 16.25;
    private static final double TAG_HEIGHT_INCHES = 29.5;
    private static final double MOUNT_ANGLE_DEGREES = 10.6;
    private static final double SHOOTER_HEIGHT = 17.7;
    private static final double BASKET_HEIGHT = 43.0;
    private static final double ENTRY_ANGLE = -45.0;
    private static final double SERVO_LOW_POS = 0.5;
    private static final double SERVO_HIGH_POS = 0.58;
    private static final double SERVO_THRESHOLD_DIST = 90.0;
    private static final double FLICKER_THRESHOLD_SEC = 1.0;
    private static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    private static final double FLYWHEEL_RADIUS = 1.89;
    private static final double SPEED_SCALAR = 3.0;
    private static final double G = 386.1;
    private static final double RPM_TOL = 150.0;

    private int[] targetIds = new int[] {20, 24};

    private final DcMotorEx leftFlywheel;
    private final DcMotorEx rightFlywheel;
    private final Servo hoodServo;
    private final Servo leftLift;
    private final Servo rightLift;

    private double targetLeftVel = 0.0;
    private double targetRightVel = 0.0;
    private double lastDistance = 0.0;
    private int lastTargetId = -1;
    private double lastTargetRPM = 0.0;
    private double lastServoPos = SERVO_LOW_POS;
    private boolean hasTargetMemory = false;
    private double lastSeenTimeSec = 0.0;

    public VisionAlignmentSubsystem(Limelight3A limelight,
                                    Follower follower,
                                    DcMotorEx leftFlywheel,
                                    DcMotorEx rightFlywheel,
                                    Servo hoodServo,
                                    Servo leftLift,
                                    Servo rightLift) {
        this.limelight = limelight;
        this.follower = follower;
        this.leftFlywheel = leftFlywheel;
        this.rightFlywheel = rightFlywheel;
        this.hoodServo = hoodServo;
        this.leftLift = leftLift;
        this.rightLift = rightLift;
    }

    /** Returns TRUE when aligned within tolerance while driving with optional auto-rotate. */
    public boolean update(double forward,
                          double strafe,
                          double rotate,
                          boolean autoAlign,
                          boolean shootEnabled) {

        if (!poseReady()) return false;

        boolean aligned = false;
        LLResultTypes.FiducialResult tag = null;

        LLResult result = null;
        if (autoAlign || shootEnabled) {
            result = limelight.getLatestResult();
        }

        if (autoAlign) {
            if (result != null && result.isValid()) {
                // Align to Limelight's best target X (tx) for reliable heading correction.
                error = goalX - result.getTx();

                tag = findTag(result, targetIds);
                if (tag != null) {
                    lastTargetId = (int) tag.getFiducialId();
                } else {
                    lastTargetId = -1;
                }

                if (Math.abs(error) < angleTolerance) {
                    rotate = 0.0;
                    aligned = true;
                } else {
                    double curTime = getTimeSec();
                    double dT = curTime - lastTime;
                    double dTerm = (dT > 0.0) ? ((error - lastError) / dT) * kD : 0.0;
                    double pTerm = error * kP;
                    rotate = clamp(pTerm + dTerm, 0.4);

                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                resetController();
            }
        } else {
            resetController();
        }

        lastRotateCmd = rotate;

        if (shootEnabled && tag == null && result != null && result.isValid()) {
            tag = findTag(result, targetIds);
        }

        if (shootEnabled) {
            runSmartShooter(tag);
            if (aligned && flywheelReady()) {
                leftLift.setPosition(0.62);
                rightLift.setPosition(0.30);
            } else {
                resetLifts();
            }
        } else {
            stopShooter();
        }

        try {
            follower.setTeleOpDrive(
                    forward,
                    strafe,
                    rotate,
                    true
            );
        } catch (NullPointerException ignored) {
            return false; // Pedro pose not latched yet
        }

        return aligned;
    }

    public void stop() {
        stopShooter();
        if (!poseReady()) return;
        try {
            follower.setTeleOpDrive(0, 0, 0, true);
        } catch (NullPointerException ignored) {}
    }

    public void setTargetIds(int... ids) {
        if (ids == null || ids.length == 0) return;
        targetIds = ids;
    }

    public double getKP() {
        return kP;
    }

    public double getKD() {
        return kD;
    }

    public double getError() {
        return error;
    }

    public double getLastRotateCmd() {
        return lastRotateCmd;
    }

    public int getLastTargetId() {
        return lastTargetId;
    }

    public double getLastDistance() {
        return lastDistance;
    }

    public double getTargetLeftVel() {
        return targetLeftVel;
    }

    public double getLastTargetRPM() {
        return lastTargetRPM;
    }

    public double getLastServoPos() {
        return lastServoPos;
    }

    public void adjustKP(double delta) {
        kP += delta;
    }

    public void adjustKD(double delta) {
        kD += delta;
    }

    public void setGoalX(double goalX) {
        this.goalX = goalX;
    }

    public void setAngleTolerance(double angleTolerance) {
        this.angleTolerance = angleTolerance;
    }

    private void resetController() {
        lastError = 0.0;
        lastTime = getTimeSec();
    }

    private boolean poseReady() {
        if (follower == null) return false;
        Pose p = follower.getPose();
        return p != null && !Double.isNaN(p.getHeading());
    }

    private LLResultTypes.FiducialResult findTag(LLResult result, int[] targets) {
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = (int) tag.getFiducialId();
            for (int target : targets) {
                if (id == target) return tag;
            }
        }
        return null;
    }

    private double getTimeSec() {
        return System.nanoTime() * 1.0e-9;
    }

    private double clamp(double v, double max) {
        return Math.max(-max, Math.min(max, v));
    }

    private void runSmartShooter(LLResultTypes.FiducialResult tag) {
        double now = getTimeSec();

        if (tag != null) {
            lastSeenTimeSec = now;
            hasTargetMemory = true;
            lastTargetId = (int) tag.getFiducialId();

            double ty = tag.getTargetYDegrees();
            double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEGREES + ty);
            lastDistance = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

            double heightY = BASKET_HEIGHT - SHOOTER_HEIGHT;
            ShotData shot = calculateShot(lastDistance, heightY, ENTRY_ANGLE);

            if (shot.isPossible) {
                lastServoPos = (lastDistance < SERVO_THRESHOLD_DIST) ? SERVO_LOW_POS : SERVO_HIGH_POS;

                lastTargetRPM = (shot.launchVelocityInchesPerSec * 60) / (2 * Math.PI * FLYWHEEL_RADIUS);
                lastTargetRPM *= SPEED_SCALAR;

                targetLeftVel = (lastTargetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;
                targetRightVel = targetLeftVel;

                if (lastDistance > 120.0) {
                    targetLeftVel *= 1.80;
                }

                hoodServo.setPosition(lastServoPos);
                leftFlywheel.setVelocity(targetLeftVel);
                rightFlywheel.setVelocity(targetRightVel);
                return;
            }
        }

        if (hasTargetMemory && (now - lastSeenTimeSec) < FLICKER_THRESHOLD_SEC) {
            hoodServo.setPosition(lastServoPos);
            leftFlywheel.setVelocity(targetLeftVel);
            rightFlywheel.setVelocity(targetRightVel);
            return;
        }

        lastTargetId = -1;
        lastDistance = 0.0;
        targetLeftVel = 0.0;
        targetRightVel = 0.0;
        lastTargetRPM = 0.0;
        hasTargetMemory = false;
        hoodServo.setPosition(SERVO_LOW_POS);
        leftFlywheel.setVelocity(0);
        rightFlywheel.setVelocity(0);
    }

    private boolean flywheelReady() {
        return Math.abs(leftFlywheel.getVelocity() - targetLeftVel) < RPM_TOL &&
                Math.abs(rightFlywheel.getVelocity() - targetRightVel) < RPM_TOL &&
                targetLeftVel > 100;
    }

    private void stopShooter() {
        leftFlywheel.setVelocity(0);
        rightFlywheel.setVelocity(0);
        hoodServo.setPosition(SERVO_LOW_POS);
        resetLifts();
        targetLeftVel = 0.0;
        targetRightVel = 0.0;
        lastTargetRPM = 0.0;
        hasTargetMemory = false;
    }

    private void resetLifts() {
        leftLift.setPosition(1.0);
        rightLift.setPosition(0.01);
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

        if (denominator <= 0) {
            data.isPossible = false;
        } else {
            data.isPossible = true;
            data.launchVelocityInchesPerSec = Math.sqrt(numerator / denominator);
        }
        return data;
    }

    private static class ShotData {
        public double launchAngleDegrees;
        public double launchVelocityInchesPerSec;
        public boolean isPossible;
    }
}
