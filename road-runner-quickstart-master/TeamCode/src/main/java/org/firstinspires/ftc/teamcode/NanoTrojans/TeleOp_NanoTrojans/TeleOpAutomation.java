package org.firstinspires.ftc.teamcode.NanoTrojans.TeleOp_NanoTrojans;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.Control_Base_NT;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;

@TeleOp(name = "TeleOpAutomation", group = "TeleOp")
public class TeleOpAutomation extends LinearOpMode {

    // --- HARDWARE ---
    private Follower follower;
    private DcMotorEx LflywheelMotor, RflywheelMotor, spindexerMotor;
    private DcMotor intakeMotor;
    private Servo llift, rlift, hoodServo;
    private Limelight3A limelight;
    private colorsensors bench;
    private ColorSensor intakeSensor;

    // --- CONSTANTS ---
    final double SPINDEXER_KP = 0.003;
    final double SPINDEXER_KF = 0.075;
    final double TICKS_PER_COMPARTMENT = 250.5;
    final double FLYWHEEL_TICKS_PER_REV = 28.0;
    final double FLYWHEEL_RADIUS = 1.89;
    final double SHOOTER_HEIGHT = 17.7;
    final double BASKET_HEIGHT = 43.0;
    final double CAMERA_HEIGHT_INCHES = 16.25;
    final double TAG_HEIGHT_INCHES = 29.5;
    final double MOUNT_ANGLE_DEGREES = 10.6;
    final double RPM_TOLERANCE = 150.0;
    final double SPEED_SCALAR = 3;
    private static final double G = 386.1;

    // --- SAFETY CONSTANT ---
    // Time (ms) to wait for lifts to physically lower before spinning spindexer
    final long LIFT_SAFETY_DELAY_MS = 450;

    // --- STATE MACHINE ENUMS ---
    enum IntakeState { STOPPED, SCANNING, INDEXING }
    enum ShootState { STOPPED, AIMING_AND_SPINUP, FIRING, RESETTING_LIFTS, INDEXING_NEXT }

    // --- VARIABLES ---
    private IntakeState intakeState = IntakeState.STOPPED;
    private ShootState shootState = ShootState.STOPPED;

    private int currentSpindexerSlot = 0;
    private int spindexerTargetPos = 0;
    private ElapsedTime spindexerCooldown = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    // Safety Timer for Lifts
    private long lastLiftResetTime = 0; // System timestamp when we last sent lifts down

    private double targetLeftVel = 0;
    private double targetRightVel = 0;

    private Control_Base_NT driveControl;

    private boolean liftIsUp = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        driveControl = new Control_Base_NT(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // --- A. DRIVER CONTROLS (GAMEPAD 1) ---
            /*follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
            follower.update();*/

            driveControl.driveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

            // --- B. AUTOMATION CONTROLS (GAMEPAD 2) ---
            handleInputGamepad2();

            // --- C. STATE UPDATES ---
            updateIntakeLogic();
            updateShooterLogic();

            // --- D. SPINDEXER CONTROL (With Safety Check) ---
            applyPIDToSpindexer();

            // --- E. TELEMETRY ---
            telemetry.addData("Intake State", intakeState);
            telemetry.addData("Shoot State", shootState);
            telemetry.addData("Lift Safe?", isLiftSafeToSpin());
            telemetry.update();
        }
    }

    // =========================================================
    //  INPUT HANDLER
    // =========================================================
    private void handleInputGamepad2() {
        // 1. L1 - START INTAKE
        if (gamepad2.left_bumper) {
            shootState = ShootState.STOPPED;
            stopShooterHardware();

            // Resets lifts immediately so they are down before we start intake
            resetLifts();

            intakeState = IntakeState.SCANNING;
            spindexerCooldown.reset();
        }

        // 2. R1 - STOP INTAKE
        if (gamepad2.right_bumper) {
            intakeState = IntakeState.STOPPED;
            intakeMotor.setPower(0);
        }

        // 3. L2 - START AUTO SHOOT
        if (gamepad2.left_trigger > 0.1) {
            intakeState = IntakeState.STOPPED;
            intakeMotor.setPower(0);

            if (shootState == ShootState.STOPPED) {
                shootState = ShootState.AIMING_AND_SPINUP;
                limelight.pipelineSwitch(0);
            }
        }

        // 4. R2 - STOP AUTO SHOOT
        if (gamepad2.right_trigger > 0.1) {
            shootState = ShootState.STOPPED;
            stopShooterHardware();
            resetLifts();
        }

        // 5. Dpad Down - MANUAL RESET
        if (gamepad2.dpad_down) {
            resetLifts();
        }

        // 6. Left Stick Y - MANUAL REVERSE (Unjam)
        // Up on stick is usually negative Y
        if (gamepad2.left_stick_y < -0.1) {
            intakeState = IntakeState.STOPPED;
            intakeMotor.setPower(gamepad2.left_stick_y); // Negative power (reverse)
        } else if (gamepad2.left_stick_y > 0.1) {
            intakeState = IntakeState.STOPPED;
            intakeMotor.setPower(gamepad2.left_stick_y); // Manual Forward
        } else if (intakeState == IntakeState.STOPPED) {
            intakeMotor.setPower(0);
        }
    }

    // =========================================================
    //  LOGIC: INTAKE AUTOMATION
    // =========================================================
    private void updateIntakeLogic() {
        switch (intakeState) {
            case SCANNING:
                intakeMotor.setPower(1);

                // Only index if cooldown passed AND lifts are physically safe
                if (spindexerCooldown.milliseconds() > 1000 && isLiftSafeToSpin()) {
                    if (intakeHasSample()) {
                        currentSpindexerSlot++;
                        sleep(200);  //give sometime before rotate
                        intakeState = IntakeState.INDEXING;
                    }
                }
                break;

            case INDEXING:
                // Motor power is handled by applyPIDToSpindexer()
                // Wait for error to be small
                double error = (currentSpindexerSlot * TICKS_PER_COMPARTMENT) - spindexerMotor.getCurrentPosition();

                if (Math.abs(error) < 15) {
                    spindexerCooldown.reset();
                    // Assuming 3 slots (0, 1, 2). If filled, stop.
                    if (currentSpindexerSlot >= 3) {
                        intakeState = IntakeState.STOPPED;
                        intakeMotor.setPower(0);
                    } else {
                        intakeState = IntakeState.SCANNING;
                    }
                }
                break;

            case STOPPED:
                break;
        }
    }

    // =========================================================
    //  LOGIC: SHOOTER AUTOMATION (NON-BLOCKING)
    // =========================================================
    private void updateShooterLogic() {
        if (shootState == ShootState.STOPPED) return;

        autoAimAndSpinUp();

        switch (shootState) {
            case AIMING_AND_SPINUP:
                if (isFlywheelReady()) {
                    shootTimer.reset();
                    shootState = ShootState.FIRING;
                }
                break;

            case FIRING:
                // Raise lifts to shoot
                /*llift.setPosition(0.625);
                rlift.setPosition(0.3);
                liftIsUp = true;*/
                moveUpLifts();

                // Wait for servo to move up and ball to launch (400ms)
                //if (shootTimer.milliseconds() > 400) {
                sleep(300);
                resetLifts(); // Command Down

                    //shootState = ShootState.RESETTING_LIFTS;
                sleep(500);
                    //rotate
                //applyPIDToSpindexer();

                double error = 100;
                double pTerm = error * SPINDEXER_KP;
                double fTerm = Math.signum(error) * SPINDEXER_KF;
                double power = pTerm + fTerm;
                power = Math.max(-0.6, Math.min(0.6, power));
                spindexerMotor.setPower(power);
                sleep(300);
                spindexerMotor.setPower(0);
                moveUpLifts();
                    // Wait for servo to move up and ball to launch (400ms)
                sleep(500);
                resetLifts(); // Command Down
                sleep(800);


                //LflywheelMotor.setPower(0);
                //RflywheelMotor.setPower(0);

                shootState = ShootState.STOPPED;

                break;

           /* case RESETTING_LIFTS:
                // WAIT until the lift timer expires
                // resetLifts() sets 'lastLiftResetTime'
                // isLiftSafeToSpin() checks that time + delay

                if (isLiftSafeToSpin()) {
                    // Once safe, check if we are empty
                    if (areChambersEmpty()) {
                        shootState = ShootState.STOPPED;
                        stopShooterHardware();
                    } else {
                        // Move to next ammo
                        currentSpindexerSlot++;
                        shootState = ShootState.INDEXING_NEXT;
                    }
                }
                break;

            case INDEXING_NEXT:
                // Wait for spindexer to settle
                double error = (currentSpindexerSlot * TICKS_PER_COMPARTMENT) - spindexerMotor.getCurrentPosition();
                if (Math.abs(error) < 20) {
                    shootState = ShootState.AIMING_AND_SPINUP;
                }
                break;*/
            case STOPPED:
                break;


        }
    }

    // =========================================================
    //  SAFETY & HARDWARE HELPERS
    // =========================================================

    /**
     * Commands lifts down and updates the safety timestamp.
     */
    private void resetLifts() {
        llift.setPosition(1);      // Down position
        rlift.setPosition(0.005);  // Down position
        liftIsUp = false;
        //lastLiftResetTime = System.currentTimeMillis();
    }

    private void moveUpLifts() {
        llift.setPosition(0.625);
        rlift.setPosition(0.3);
        liftIsUp = true;
        //lastLiftResetTime = System.currentTimeMillis();
    }

    private void moveUpLeftLift() {
        llift.setPosition(0.625);
        //rlift.setPosition(0.3);
        liftIsUp = true;
        //lastLiftResetTime = System.currentTimeMillis();
    }

    private void moveUpRightLift() {
        //llift.setPosition(0.625);
        rlift.setPosition(0.3);
        liftIsUp = true;
        //lastLiftResetTime = System.currentTimeMillis();
    }

    /**
     * Returns true ONLY if enough time has passed since resetLifts()
     * for the servos to physically be out of the way.
     */
    private boolean isLiftSafeToSpin() {
        return (System.currentTimeMillis() - lastLiftResetTime) > LIFT_SAFETY_DELAY_MS;
    }

    private void applyPIDToSpindexer() {
        // SAFETY OVERRIDE:
        // If the lifts were commanded down recently, DO NOT MOVE Spindexer.
        // This prevents the carousel from hitting the lifts while they are lowering.
        /*if (!isLiftSafeToSpin()) {
            spindexerMotor.setPower(0);
            return;
        }*/
        if(liftIsUp) {
            spindexerMotor.setPower(0);
            return;
        }

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

    // --- OTHER HELPERS ---

    private void stopShooterHardware() {
        LflywheelMotor.setVelocity(0);
        RflywheelMotor.setVelocity(0);
        hoodServo.setPosition(0.5);
    }

    private boolean intakeHasSample() {
        if (intakeSensor == null) return false;
        double totalColor = intakeSensor.red() + intakeSensor.green() + intakeSensor.blue();
        return totalColor > 300;
    }

    private boolean areChambersEmpty() {
        colorsensors.DetectedColor l = bench.detectByHue(bench.left, telemetry);
        colorsensors.DetectedColor r = bench.detectByHue(bench.right, telemetry);
        return (l == colorsensors.DetectedColor.UNKNOWN && r == colorsensors.DetectedColor.UNKNOWN);
    }

    private boolean isFlywheelReady() {
        double curL = LflywheelMotor.getVelocity();
        double curR = RflywheelMotor.getVelocity();
        boolean speedGood = Math.abs(curL - targetLeftVel) < RPM_TOLERANCE && Math.abs(curR - targetRightVel) < RPM_TOLERANCE;
        return speedGood && targetLeftVel > 100;
    }

    private void autoAimAndSpinUp() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE_DEGREES + ty);
            double currentDist = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

            double heightY = BASKET_HEIGHT - SHOOTER_HEIGHT;
            ShotData shot = calculateShot(currentDist, heightY, -45);

            if (shot.isPossible) {
                if (currentDist < 90.0) hoodServo.setPosition(0.5);
                else hoodServo.setPosition(0.58);

                double targetRPM = (shot.launchVelocityInchesPerSec * 60) / (2 * Math.PI * FLYWHEEL_RADIUS);
                targetRPM *= SPEED_SCALAR;
                double velTicks = (targetRPM / 60.0) * FLYWHEEL_TICKS_PER_REV;

                targetLeftVel = velTicks;
                targetRightVel = velTicks;
                if (currentDist > 120.0) targetLeftVel *= 1.80;

                LflywheelMotor.setVelocity(targetLeftVel);
                RflywheelMotor.setVelocity(targetRightVel);
            }
        }
    }

    private ShotData calculateShot(double x, double y, double theta) {
        ShotData data = new ShotData();
        double thetaRad = Math.toRadians(theta);
        double term1 = (2 * y) / x;
        double term2 = Math.tan(thetaRad);
        double alphaRad = Math.atan(term1 - term2);

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
        public double launchVelocityInchesPerSec;
        public boolean isPossible;
    }

    private void initHardware() {
        LflywheelMotor = hardwareMap.get(DcMotorEx.class, "lgun");
        LflywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(70, 0, 1.5, 13.5));

        RflywheelMotor = hardwareMap.get(DcMotorEx.class, "rgun");
        RflywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RflywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RflywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(70, 0, 1.5, 13.5));

        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        llift = hardwareMap.get(Servo.class, "llift");
        rlift = hardwareMap.get(Servo.class, "rlift");

        // Reset immediately
        resetLifts();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        bench = new colorsensors();
        bench.init(hardwareMap);
        intakeSensor = hardwareMap.get(ColorSensor.class, "intake sensor");
    }
}