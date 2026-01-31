package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * ==========================================================================================
 * NANO TROJANS - FAR RED AUTO (FULL LIBRARY INTEGRATION)
 * ==========================================================================================
 * Features:
 * 1. Logic: TeleOp "Fire All" Sequence.
 * 2. Hardware: Uses 'colorsensors.java' for accurate Hue-based detection.
 * 3. Telemetry: Displays 'GREEN', 'PURPLE', or 'UNKNOWN' from the library.
 * ==========================================================================================
 */

@Autonomous(name = "Close Red Auto - Final Integration", group = "Competition")
public class CloseRedAuto3 extends LinearOpMode {

    // --- HARDWARE ---
    private Follower follower;
    private Paths paths;
    private DcMotorEx LflywheelMotor, RflywheelMotor;
    private Servo hoodServo;
    private DcMotorEx spindexerMotor;
    private DcMotor intakeMotor;
    private Servo llift, rlift;
    private Limelight3A limelight;

    // Sensors
    private ColorSensor intakeSensor; // Used for raw threshold checks logic
    private colorsensors bench;       // Custom Library for Hue Logic

    // --- CONSTANTS ---
    final double SPINDEXER_KP = 0.003;
    final double TICKS_PER_COMPARTMENT = 250.5;
    final double RPM_TOLERANCE = 150.0;
    final double INTAKE_THRESHOLD = 300;

    // Servo Positions
    final double SERVO_LOW_POS  = 0.5;
    final double SERVO_HIGH_POS = 0.58;
    final double SERVO_SHOOT_POS_L = 0.625;
    final double SERVO_SHOOT_POS_R = 0.3;
    final double SERVO_IDLE_POS_L = 1.0;
    final double SERVO_IDLE_POS_R = 0.005;

    // --- STATE ENUMS ---
    private enum IntakeState { STOPPED, SCANNING, INDEXING }
    private enum ShootState {
        STOPPED, AIMING_AND_SPINUP, FIRING_LIFT_UP_1, FIRING_RESET_1,
        FIRING_SPIN_INDEXER, FIRING_LIFT_UP_2, FIRING_RESET_2
    }

    // --- VARIABLES ---
    private IntakeState intakeState = IntakeState.STOPPED;
    private ShootState shootState = ShootState.STOPPED;

    private int detectedApriltagId = -1;
    private int currentSpindexerSlot = 0;

    private double targetLeftVel = 0;
    private double targetRightVel = 0;
    private double lastKnownDistance = 0;

    private ElapsedTime spindexerCooldown = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(122.600, 122.000, Math.toRadians(36.4)));
        paths = new Paths(follower);

        telemetry.addData("Status", "Initialized. Bench Library Active.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            limelight.start();
            resetLifts();


            // 2. PATH 1 -> SHOOT
            driveWithStates(paths.Path1, false, false, 3000);
            scanObeliskWithTimeout(2000);
            driveWithStates(paths.Path2, false, false, 3000);
            performTeleOpShootingSequence();

            /*// 3. PATH 2 -> PATH 3 (Intake ON)
            driveWithStates(paths.Path2, false, false, 3000);
            driveWithStates(paths.Path3, true, false, 5000);

            // 4. PATH 4 -> SHOOT
            driveWithStates(paths.Path4, false, false, 4000);
            performTeleOpShootingSequence();

            // 5. PATH 5 -> PATH 6 (Intake ON)
            driveWithStates(paths.Path5, false, false, 3000);
            driveWithStates(paths.Path6, true, false, 5000);

            // 6. PATH 7 -> FINAL SHOOT
            driveWithStates(paths.Path7, false, false, 4000);
            performTeleOpShootingSequence();
*/
            stopShooterHardware();

            // TELEMETRY HOLD LOOP
            while (opModeIsActive()) {
                telemetry.addData("STATUS", "MISSION COMPLETE");
                updateMasterTelemetry(-1, -1);
            }
        }
    }

    // =========================================================================
    //   TELEOP SHOOTING LOGIC
    // =========================================================================

    private void performTeleOpShootingSequence() {
        shootState = ShootState.AIMING_AND_SPINUP;
        shootTimer.reset();

        while (opModeIsActive() && shootState != ShootState.STOPPED) {
            follower.update(); // Maintain Position
            autoAimAndSpinUp();

            switch (shootState) {
                case AIMING_AND_SPINUP:
                    if (isFlywheelReady()) {
                        shootTimer.reset();
                        shootState = ShootState.FIRING_LIFT_UP_1;
                    }
                    if (shootTimer.milliseconds() > 1500) { // Timeout
                        shootTimer.reset();
                        shootState = ShootState.FIRING_LIFT_UP_1;
                    }
                    break;

                case FIRING_LIFT_UP_1:
                    moveUpLifts();
                    if (shootTimer.milliseconds() > 300) {
                        shootTimer.reset();
                        shootState = ShootState.FIRING_RESET_1;
                    }
                    break;

                case FIRING_RESET_1:
                    resetLifts();
                    if (shootTimer.milliseconds() > 500) {
                        shootTimer.reset();
                        shootState = ShootState.FIRING_SPIN_INDEXER;
                    }
                    break;

                case FIRING_SPIN_INDEXER:
                    spindexerMotor.setPower(0.375); // The Nudge
                    if (shootTimer.milliseconds() > 300) {
                        spindexerMotor.setPower(0);
                        shootTimer.reset();
                        shootState = ShootState.FIRING_LIFT_UP_2;
                    }
                    break;

                case FIRING_LIFT_UP_2:
                    moveUpLifts();
                    if (shootTimer.milliseconds() > 500) {
                        shootTimer.reset();
                        shootState = ShootState.FIRING_RESET_2;
                    }
                    break;

                case FIRING_RESET_2:
                    resetLifts();
                    if (shootTimer.milliseconds() > 800) {
                        shootState = ShootState.STOPPED;
                    }
                    break;
            }
            updateMasterTelemetry(-1, -1);
        }
        stopShooterHardware();
    }

    // =========================================================================
    //   DRIVE & INTAKE LOGIC
    // =========================================================================

    private void driveWithStates(PathChain path, boolean intakeActive, boolean shooterActive, double timeoutMs) {
        follower.followPath(path, true);
        ElapsedTime timer = new ElapsedTime();

        if (intakeActive) {
            intakeState = IntakeState.SCANNING;
            spindexerCooldown.reset();
        } else {
            intakeState = IntakeState.STOPPED;
        }

        while (opModeIsActive() && follower.isBusy() && timer.milliseconds() < timeoutMs) {
            follower.update();
            updateIntakeLogic();
            if (!shooterActive) stopShooterHardware();
            updateMasterTelemetry(timer.milliseconds(), timeoutMs);
        }

        if (timer.milliseconds() >= timeoutMs) follower.breakFollowing();
        intakeMotor.setPower(0);
    }

    private void updateIntakeLogic() {
        if (shootState == ShootState.FIRING_SPIN_INDEXER) return;

        applyPIDToSpindexer();

        switch (intakeState) {
            case SCANNING:
                intakeMotor.setPower(1.0);
                if (spindexerCooldown.milliseconds() > 1000 && intakeHasSample()) {
                    currentSpindexerSlot++;
                    intakeState = IntakeState.INDEXING;
                }
                break;

            case INDEXING:
                intakeMotor.setPower(1.0);
                double target = currentSpindexerSlot * TICKS_PER_COMPARTMENT;
                if (Math.abs(target - spindexerMotor.getCurrentPosition()) < 15) {
                    spindexerCooldown.reset();
                    intakeState = IntakeState.SCANNING;
                }
                break;

            case STOPPED:
                intakeMotor.setPower(0);
                break;
        }
    }

    // =========================================================================
    //   TELEMETRY & COLOR HELPERS (FIXED)
    // =========================================================================

    private void updateMasterTelemetry(double currentTime, double maxTime) {
        String obeliskStatus = (detectedApriltagId == -1) ? "Scanning..." : ("ID " + detectedApriltagId);
        telemetry.addData("1. Obelisk", obeliskStatus);

        // --- UPDATED: CORRECT LIBRARY METHOD CALLS ---
        // Using the exact names found in 'colorsensors.java'
        colorsensors.DetectedColor lEnum = bench.getDetectedColor(telemetry);
        colorsensors.DetectedColor rEnum = bench.getrightcolor(telemetry);
        colorsensors.DetectedColor inEnum = bench.getbackcolor(telemetry);

        telemetry.addData("2. Sensors", "L:[%s] R:[%s] In:[%s]",
                lEnum.toString(), rEnum.toString(), inEnum.toString());

        double currentRPM = (LflywheelMotor.getVelocity() / 28.0) * 60;
        double targetRPM = (targetLeftVel / 28.0) * 60;
        telemetry.addData("3. Shooter", "Dist: %.1f | RPM: %.0f / %.0f", lastKnownDistance, currentRPM, targetRPM);
        telemetry.addData("4. State", "Shoot:%s | Intake:%s", shootState, intakeState);

        if (currentTime != -1) telemetry.addData("5. Timer", "%.1f / %.1f s", currentTime/1000, maxTime/1000);
        telemetry.update();
    }

    // =========================================================================
    //   HELPERS
    // =========================================================================

    private void moveUpLifts() {
        llift.setPosition(SERVO_SHOOT_POS_L);
        rlift.setPosition(SERVO_SHOOT_POS_R);
    }

    private void resetLifts() {
        llift.setPosition(SERVO_IDLE_POS_L);
        rlift.setPosition(SERVO_IDLE_POS_R);
    }

    private boolean isFlywheelReady() {
        double curL = LflywheelMotor.getVelocity();
        double curR = RflywheelMotor.getVelocity();
        return Math.abs(curL - targetLeftVel) < RPM_TOLERANCE && Math.abs(curR - targetRightVel) < RPM_TOLERANCE;
    }

    private void applyPIDToSpindexer() {
        double target = currentSpindexerSlot * TICKS_PER_COMPARTMENT;
        double error = target - spindexerMotor.getCurrentPosition();
        double power = (error * SPINDEXER_KP) + (Math.signum(error) * 0.075);
        if (power > 0.6) power = 0.6; if (power < -0.6) power = -0.6;
        spindexerMotor.setPower(power);
    }

    private void autoAimAndSpinUp() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            lastKnownDistance = (29.5 - 16.25) / Math.tan(Math.toRadians(10.6 + ty));

            hoodServo.setPosition(lastKnownDistance < 90 ? SERVO_LOW_POS : SERVO_HIGH_POS);

            double rpm = (lastKnownDistance > 72) ? 3200 : 2500;
            targetLeftVel = (rpm / 60.0) * 28.0;
            targetRightVel = targetLeftVel;

            LflywheelMotor.setVelocity(targetLeftVel);
            RflywheelMotor.setVelocity(targetRightVel);
        }
    }

    private void scanObeliskWithTimeout(int ms) {
        ElapsedTime t = new ElapsedTime();
        while(opModeIsActive() && t.milliseconds() < ms) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    int id = tag.getFiducialId();
                    if (id >= 21 && id <= 23) {
                        detectedApriltagId = id;
                        telemetry.addData("ID", id);
                        telemetry.update();
                        return;
                    }
                }
            }
            updateMasterTelemetry(t.milliseconds(), ms);
        }
    }

    private boolean intakeHasSample() {
        if (intakeSensor == null) return false;
        // Check total brightness vs threshold
        return (intakeSensor.alpha() > INTAKE_THRESHOLD);
    }

    private void stopShooterHardware() {
        LflywheelMotor.setVelocity(0);
        RflywheelMotor.setVelocity(0);
    }

    private void initHardware() {
        // 1. Initialize Library (Critical for LED/Gain config)
        bench = new colorsensors();
        bench.init(hardwareMap);

        // 2. Initialize Manual References (For our read logic)
        // Note: 'bench.init' already mapped the color sensors, but we grab one for 'intakeHasSample' logic
        intakeSensor = hardwareMap.get(ColorSensor.class, "intake sensor");

        LflywheelMotor = hardwareMap.get(DcMotorEx.class, "lgun");
        RflywheelMotor = hardwareMap.get(DcMotorEx.class, "rgun");
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexer");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        llift = hardwareMap.get(Servo.class, "llift");
        rlift = hardwareMap.get(Servo.class, "rlift");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        LflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RflywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetLifts();
    }
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;

        public Paths(Follower follower) {

            // Path 1: Start (Top Right) -> Shooting Spot
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(122.600, 122.000),
                                    new Pose(90.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36.4), Math.toRadians(112))
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