package org.firstinspires.ftc.teamcode.NanoTrojans.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.NanoTrojans.Subsystems.VisionAlignmentSubsystem;

@TeleOp(name = "Test - Vision Align + Shooter", group = "Competition")
public class TestVisionAlignmentSystem extends LinearOpMode {

    private Follower follower;
    private Limelight3A limelight;
    private VisionAlignmentSubsystem aligner;

    private DcMotorEx lGun, rGun;
    private Servo hood;
    private Servo llift, rlift;

    // -------------------- controller based PD tuning -----------------------
    private final double[] stepSizes = {0.1, 0.001, 0.0001};
    private int stepIndex = 1;

    @Override
    public void runOpMode() {

        // ---- PEDRO FOLLOWER (TUNED) ----
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Force localizer latch
        for (int i = 0; i < 30; i++) follower.update();

        // ---- LIMELIGHT ----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // ---- SHOOTER ----
        lGun = hardwareMap.get(DcMotorEx.class, "lgun");
        rGun = hardwareMap.get(DcMotorEx.class, "rgun");
        hood = hardwareMap.get(Servo.class, "hood");
        llift = hardwareMap.get(Servo.class, "llift");
        rlift = hardwareMap.get(Servo.class, "rlift");

        lGun.setDirection(DcMotorSimple.Direction.FORWARD);
        rGun.setDirection(DcMotorSimple.Direction.REVERSE);
        lGun.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rGun.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(70, 0, 1.5, 13.5);
        lGun.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rGun.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // ---- SUBSYSTEMS ----
        aligner = new VisionAlignmentSubsystem(limelight, follower, lGun, rGun, hood, llift, rlift);
        aligner.setTargetIds(20, 24);

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            Pose pose = follower.getPose();
            boolean poseOK = pose != null && !Double.isNaN(pose.getHeading());

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            boolean autoAlign = gamepad1.left_trigger > 0.3;

            if (poseOK) {
                // Align only for now; shooter disabled
                aligner.update(forward, strafe, rotate, autoAlign, false);
            } else {
                aligner.stop();
            }

            // update P and D on the fly (same logic as the reference OpMode)
            if (gamepad1.bWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            if (gamepad1.dpadLeftWasPressed()) {
                aligner.adjustKP(-stepSizes[stepIndex]);
            }
            if (gamepad1.dpadRightWasPressed()) {
                aligner.adjustKP(stepSizes[stepIndex]);
            }

            if (gamepad1.dpadUpWasPressed()) {
                aligner.adjustKD(stepSizes[stepIndex]);
            }
            if (gamepad1.dpadDownWasPressed()) {
                aligner.adjustKD(-stepSizes[stepIndex]);
            }

            // Telemetry
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid()) {
                if (autoAlign) {
                    telemetry.addLine("AUTO ALIGN");
                }
                telemetry.addData("Tx", r.getTx());
                telemetry.addData("Error", aligner.getError());
                telemetry.addData("TurnCmd", "%.3f", aligner.getLastRotateCmd());
                telemetry.addData("Tag", aligner.getLastTargetId());
                telemetry.addData("Dist", "%.1f", aligner.getLastDistance());
                telemetry.addData("Hood", "%.2f", aligner.getLastServoPos());
                telemetry.addData("Target RPM", "%.0f", aligner.getLastTargetRPM());
            } else {
                telemetry.addLine("MANUAL Rotate Mode");
            }
            telemetry.addLine("-----------------------------");
            telemetry.addData("Tuning P", "%.4f (D-Pad L/R)", aligner.getKP());
            telemetry.addData("Tuning D", "%.4f (D-Pad U/D)", aligner.getKD());
            telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
            telemetry.update();
        }
    }
}
