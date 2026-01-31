package org.firstinspires.ftc.teamcode.NanoTrojans.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;

public class IntakeSubsystem {
    // Hardware Components
    private DcMotor intakeMotor;
    private DcMotorEx spindexerMotor;
    private ColorSensor intakeSensor;
    private colorsensors bench; // Library for gun sensors

    // Constants from Victor's Tuned Code
    private final double SPINDEXER_KP = 0.003;
    private final double SPINDEXER_KF = 0.075;
    private final double TICKS_PER_COMPARTMENT = 250.5;
    private final int POSITION_TOLERANCE = 15;

    // State Management
    private int currentSpindexerSlot = 0;
    private boolean isIndexing = false;
    private ElapsedTime spindexerCooldown = new ElapsedTime();

    public IntakeSubsystem(HardwareMap hwMap) {
        // Intake Motor Setup
        intakeMotor = hwMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        // Spindexer Motor Setup
        spindexerMotor = hwMap.get(DcMotorEx.class, "spindexer");
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sensor Setup
        intakeSensor = hwMap.get(ColorSensor.class, "intake sensor");
        bench = new colorsensors();
        bench.init(hwMap);
    }

    /**
     * Main automation loop for the Intake System.
     * Logic: Runs until all 3 chambers (Intake + 2 Gun Chambers) are full.
     */
    public void runIntakeAutomation(boolean liftIsUp) {
        // Rule: Only stop if all three sensors detect an artifact
        if (isFullyLoaded()) {
            stopIntake();
            return;
        }

        if (!isIndexing) {
            intakeMotor.setPower(1.0); // Standard intake power

            // Check if artifact entered and cooldown has passed
            if (artifactAtIntake() && spindexerCooldown.milliseconds() > 1000) {
                currentSpindexerSlot++;
                isIndexing = true;
            }
        } else {
            applySpindexerPID(liftIsUp);
        }
    }

    private void applySpindexerPID(boolean liftIsUp) {
        // Safety: Do not rotate if shooter lifts are raised
        if (liftIsUp) {
            spindexerMotor.setPower(0);
            return;
        }

        double targetPos = currentSpindexerSlot * TICKS_PER_COMPARTMENT;
        double currentPos = spindexerMotor.getCurrentPosition();
        double error = targetPos - currentPos;

        if (Math.abs(error) < POSITION_TOLERANCE) {
            spindexerMotor.setPower(0);
            isIndexing = false;
            spindexerCooldown.reset();
            return;
        }

        // PID Logic: P-term + Feedforward signum
        double power = (error * SPINDEXER_KP) + (Math.signum(error) * SPINDEXER_KF);
        power = Math.max(-0.6, Math.min(0.6, power)); // Limit to 60% power
        spindexerMotor.setPower(power);
    }

    private boolean artifactAtIntake() {
        // Threshold (Red + Green + Blue > 300) from Victor's code
        return (intakeSensor.red() + intakeSensor.green() + intakeSensor.blue()) > 300;
    }

    public boolean isFullyLoaded() {
        // Use bench library to detect artifacts in back gun chambers
        boolean leftChamberFull = bench.detectByHue(bench.left, null) != colorsensors.DetectedColor.UNKNOWN;
        boolean rightChamberFull = bench.detectByHue(bench.right, null) != colorsensors.DetectedColor.UNKNOWN;

        return leftChamberFull && rightChamberFull && artifactAtIntake();
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
        spindexerMotor.setPower(0);
    }
}