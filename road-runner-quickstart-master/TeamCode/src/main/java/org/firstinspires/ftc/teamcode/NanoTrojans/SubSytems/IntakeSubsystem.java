package org.firstinspires.ftc.teamcode.NanoTrojans.Subsystems;

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

    private final double SPINDEXER_KP = 0.003;
    private final double SPINDEXER_KF = 0.075;
    private final double TICKS_PER_COMPARTMENT = 250.5;

    private int currentSpindexerSlot = 0;
    private boolean isIndexing = false;
    private boolean lastSensorState = false; // To detect False -> True transition
    private int artifactsIntaken = 0;

    private ElapsedTime intakeSettlingTimer = new ElapsedTime();
    private boolean waitingForSettle = false;
    private final double SETTLE_TIME_MS = 350;
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

        // Detect TRANSITION from False to True
        if (currentSensorState && !lastSensorState) {
            artifactsIntaken++; // Increment the "Intook One" counter
        }
        lastSensorState = currentSensorState; // Update for next loop

        if (isFullyLoaded() || currentSpindexerSlot >= 3) {
            stopIntake();
            return;
        }

        if (isIndexing) {
            intakeMotor.setPower(0);
            applySpindexerPID(liftIsUp);
        } else {
            intakeMotor.setPower(1.0);

            // Logic: Must see artifact, bench must have room, and must not be in 1s cooldown
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

        double preciseTarget = currentSpindexerSlot * TICKS_PER_COMPARTMENT;
        double error = preciseTarget - spindexerMotor.getCurrentPosition();

        if (Math.abs(error) < 15) {
            spindexerMotor.setPower(0);
            isIndexing = false;
            spindexerCooldown.reset();
            return;
        }

        double power = (error * SPINDEXER_KP) + (Math.signum(error) * SPINDEXER_KF);
        power = Math.max(-0.45, Math.min(0.45, power));
        spindexerMotor.setPower(power);
    }

    // --- SENSOR GETTERS FOR TELEMETRY ---

    public boolean artifactDetectedAtIntake() {
        return getIntakeRawSum() > 300;
    }

    public int getIntakeRawSum() {
        return intakeSensor.red() + intakeSensor.green() + intakeSensor.blue();
    }

    public boolean isLeftBenchFull() {
        return bench.detectByHue(bench.left, null) != colorsensors.DetectedColor.UNKNOWN;
    }

    public boolean isRightBenchFull() {
        return bench.detectByHue(bench.right, null) != colorsensors.DetectedColor.UNKNOWN;
    }

    public boolean isBenchFull() {
        return isLeftBenchFull() && isRightBenchFull();
    }

    public boolean isFullyLoaded() {
        return isBenchFull() && artifactDetectedAtIntake();
    }

    public int getArtifactsIntakenCount() { return artifactsIntaken; }

    public void stopIntake() {
        intakeMotor.setPower(0);
        spindexerMotor.setPower(0);
        isIndexing = false;
    }
}