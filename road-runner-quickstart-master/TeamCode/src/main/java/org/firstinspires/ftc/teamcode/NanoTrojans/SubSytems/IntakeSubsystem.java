package org.firstinspires.ftc.teamcode.NanoTrojans.SubSytems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;

public class IntakeSubsystem {
    private DcMotor intakeMotor;
    private DcMotorEx spindexerMotor;
    private ColorSensor intakeSensor;
    private colorsensors bench;

    // --- PIDF CONSTANTS (Exactly matched to your Tuner) ---
    private double kP = 0.002;
    private double kF = 0.05;
    private double kD = 0.000;
    private double ticksPerCompartment = 250.6;

    // --- SETTINGS ---
    private final double SETTLE_TIME_MS = 350;
    private final int EXIT_TOLERANCE = 5; // Matches Tuner friction deadzone

    // --- STATE ---
    private int currentSpindexerSlot = 0;
    private boolean isIndexing = false;
    private boolean lastSensorState = false;
    private int artifactsIntaken = 0;
    private double lastError = 0;

    private ElapsedTime intakeSettlingTimer = new ElapsedTime();
    private boolean waitingForSettle = false;
    private ElapsedTime spindexerCooldown = new ElapsedTime();

    public IntakeSubsystem(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        spindexerMotor = hwMap.get(DcMotorEx.class, "spindexer");
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeSensor = hwMap.get(ColorSensor.class, "intake sensor");
        bench = new colorsensors();
        bench.init(hwMap);
    }

    public void runIntakeAutomation(boolean liftIsUp) {
        boolean currentSensorState = artifactDetectedAtIntake();

        // Detect transitions to count artifacts
        if (currentSensorState && !lastSensorState) {
            artifactsIntaken++;
        }
        lastSensorState = currentSensorState;

        if (isFullyLoaded() || currentSpindexerSlot >= 3) {
            stopIntake();
            return;
        }

        if (isIndexing) {
            intakeMotor.setPower(0); // Intake remains OFF during rotation
            applySpindexerPID(liftIsUp);
        } else {
            intakeMotor.setPower(1.0);

            if (currentSensorState && !isBenchFull() && spindexerCooldown.milliseconds() > 1000) {
                if (!waitingForSettle) {
                    intakeSettlingTimer.reset();
                    waitingForSettle = true;
                }

                if (intakeSettlingTimer.milliseconds() > SETTLE_TIME_MS) {
                    currentSpindexerSlot++;
                    isIndexing = true;
                    waitingForSettle = false;
                }
            } else {
                waitingForSettle = false;
            }
        }
    }

    private void applySpindexerPID(boolean liftIsUp) {
        if (liftIsUp) {
            spindexerMotor.setPower(0);
            return;
        }

        // 1. Calculate target using the Tuner's rounding logic
        int targetPosition = (int) Math.round(currentSpindexerSlot * ticksPerCompartment);
        int currentPos = spindexerMotor.getCurrentPosition();
        double error = targetPosition - currentPos;

        // 2. Tighter Exit Condition
        if (Math.abs(error) < EXIT_TOLERANCE) {
            spindexerMotor.setPower(0);
            isIndexing = false;
            spindexerCooldown.reset();
            return;
        }

        // 3. PIDF Math from Tuner
        double pTerm = error * kP;

        // Only apply Friction power if we are outside the 5-tick deadzone
        double fTerm = 0;
        if (Math.abs(error) > 5) fTerm = Math.signum(error) * kF;

        double dTerm = (error - lastError) * kD;
        lastError = error;

        double power = pTerm + fTerm + dTerm;

        // Cap power for safety (Tuner used 1.0, we use 0.5)
        power = Math.max(-0.5, Math.min(0.5, power));
        spindexerMotor.setPower(power);
    }

    public boolean artifactDetectedAtIntake() {
        return (intakeSensor.red() + intakeSensor.green() + intakeSensor.blue()) > 300;
    }

    public boolean isBenchFull() {
        return (bench.detectByHue(bench.left, null) != colorsensors.DetectedColor.UNKNOWN &&
                bench.detectByHue(bench.right, null) != colorsensors.DetectedColor.UNKNOWN);
    }

    public boolean isFullyLoaded() {
        return isBenchFull() && artifactDetectedAtIntake();
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
        spindexerMotor.setPower(0);
        isIndexing = false;
    }

    public int getArtifactsIntakenCount() { return artifactsIntaken; }
    // 1. Fixed the "getIntakeRawSum" error
    public int getIntakeRawSum() {
        return intakeSensor.red() + intakeSensor.green() + intakeSensor.blue();
    }

    // 2. Fixed the "isLeftBenchFull" error
    public boolean isLeftBenchFull() {
        return bench.detectByHue(bench.left, null) != colorsensors.DetectedColor.UNKNOWN;
    }

    // 3. Fixed the "isRightBenchFull" error
    public boolean isRightBenchFull() {
        return bench.detectByHue(bench.right, null) != colorsensors.DetectedColor.UNKNOWN;
    }

    // 4. Added for total awareness in the tester
    public boolean isIndexing() {
        return isIndexing;
    }
}