package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TOOL: Hood Calibrator", group = "Calibration")
public class HoodCalibrator extends LinearOpMode {

    private DcMotorEx hoodMotor;

    @Override
    public void runOpMode() {
        hoodMotor = hardwareMap.get(DcMotorEx.class, "hood");

        // Reset so "Current Position" is 0
        hoodMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hoodMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hoodMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("1. Manually push hood to LOWEST position (0 degrees).");
        telemetry.addLine("2. Press Start.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Use Gamepad Stick to move hood gently
            double power = -gamepad1.left_stick_y * 0.3; // 30% speed max
            hoodMotor.setPower(power);

            int currentTicks = hoodMotor.getCurrentPosition();

            telemetry.addData("Current Ticks", currentTicks);
            telemetry.addLine("\n--- INSTRUCTIONS ---");
            telemetry.addLine("1. Use Left Stick to move hood to exactly 45 DEGREES (use a protractor/triangle).");
            telemetry.addLine("2. Read the 'Current Ticks' number.");
            telemetry.addLine("3. Calculate: Ticks / 45 = Your Constant.");
            telemetry.update();
        }
    }
}