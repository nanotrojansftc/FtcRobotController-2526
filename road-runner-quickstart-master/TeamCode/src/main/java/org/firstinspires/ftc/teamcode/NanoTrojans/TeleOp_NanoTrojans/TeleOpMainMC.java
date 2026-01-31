package org.firstinspires.ftc.teamcode.NanoTrojans.TeleOp_NanoTrojans;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top;

import java.util.List;

@TeleOp(name ="TeleOpMainMC", group = "TeleOp")
public class TeleOpMainMC extends LinearOpMode {

    private colorsensors bench;
    private controls_top control;
    private resources_top resources;
    private Follower follower;
    public boolean llift_status = false;//False is down, true is up
    public boolean rlift_status = false;
    float lefthue;
    float righthue;
    float backhue;
    public int carousel = 0;

    private Limelight3A limelight3A;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();

        bench = new colorsensors();
        bench.init(hardwareMap);
        resources = new resources_top(hardwareMap);
        control = new controls_top(resources.lgun, resources.rgun, resources.intake, resources.llift, resources.rlift, resources.fspin, resources.rspin, resources.lspin);

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.setPollRateHz(100);
        limelight3A.start();
        limelight3A.pipelineSwitch(0);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        waitForStart();

        Thread intakeControlThread = new Thread(new intakeControl());
        Thread shootThread = new Thread(new shooting());
        Thread limelightThread = new Thread(new limelight());

        intakeControlThread.start();
        shootThread.start();
        limelightThread.start();

        while (!Thread.interrupted() && opModeIsActive()) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
            follower.update();
        }
    }

    private class intakeControl implements Runnable {
        @Override
        public void run() {
            waitForStart();
            while (!Thread.interrupted() && opModeIsActive()) {
                // FIXED: Updated to use .getHue() and removed telemetry argument
                lefthue = bench.getHue(bench.left);
                righthue = bench.getHue(bench.right);
                backhue = bench.getHue(bench.back);

                //telemetry.addData("Left color", "%s, Hue=%.2f", bench.detectByHue(lefthue, telemetry), lefthue);
                //telemetry.addData("Right color", "%s, Hue=%.2f", bench.detectByHue(righthue, telemetry), righthue);
                //telemetry.addData("Back color", "%s, Hue=%.2f", bench.detectByHue(backhue, telemetry), backhue);
                telemetry.addData("rightlift", rlift_status);
                telemetry.addData("leftlift", llift_status);
                telemetry.update();

                if(bench.detectByHue(bench.back, telemetry) == colorsensors.DetectedColor.UNKNOWN) {
                    double intakePower = gamepad2.left_stick_y;
                    resources.intake.setPower(intakePower);
                    while(bench.detectByHue(bench.back, telemetry) != colorsensors.DetectedColor.UNKNOWN) {
                        if(!(bench.detectByHue(bench.back, telemetry) != colorsensors.DetectedColor.UNKNOWN
                                && bench.detectByHue(bench.right, telemetry) != colorsensors.DetectedColor.UNKNOWN
                                && bench.detectByHue(bench.left, telemetry) != colorsensors.DetectedColor.UNKNOWN)) {
                            rotate();
                            sleep(500);
                            if(bench.detectByHue(bench.back, telemetry) == colorsensors.DetectedColor.UNKNOWN) break;
                            else {
                                rotate();
                                sleep(500);
                                if(bench.detectByHue(bench.back, telemetry) == colorsensors.DetectedColor.UNKNOWN) break;
                                else {
                                    rotate();
                                    sleep(500);
                                }
                                break;
                            }
                        }
                        break;
                    }
                }
                if(bench.detectByHue(bench.back, telemetry) != colorsensors.DetectedColor.UNKNOWN) {
                    resources.intake.setPower(0);
                }
            }
        }
    }

    public class shooting implements Runnable {
        @Override
        public void run() {
            waitForStart();
            while (!Thread.interrupted() && opModeIsActive()) {
                if (gamepad2.left_trigger > 0) {
                    resources.llift.setPosition(1);
                    llift_status = false;
                }
                if (gamepad2.left_bumper){
                    resources.llift.setPosition(0.625);
                    llift_status= true;
                }
                if (gamepad2.right_trigger > 0){
                    resources.rlift.setPosition(0.005);
                    rlift_status = false;
                }
                if (gamepad2.right_bumper){
                    resources.rlift.setPosition(0.3);
                    rlift_status = true;
                }
                if (gamepad2.dpad_left) resources.lgun.setPower(-1);
                if (gamepad2.dpad_right) resources.rgun.setPower(1);
                if (gamepad2.dpad_down) {
                    resources.lgun.setPower(0);
                    resources.rgun.setPower(0);
                }
                if (gamepad2.x) rotate();
                if (gamepad2.a) shootgreen();
                if (gamepad2.b) shootpurple();
            }
        }
    }

    public class limelight implements Runnable {
        @Override
        public void run() {
            waitForStart();
            while (!Thread.interrupted() && opModeIsActive()) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                limelight3A.updateRobotOrientation(orientation.getYaw());
                LLResult llResult = limelight3A.getLatestResult();
                if (llResult != null && llResult.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
                    //telemetry.addData("AprilTags Detected", fiducials.size());
                    for (LLResultTypes.FiducialResult tag : fiducials) {
                        //telemetry.addData("AprilTag ID", tag.getFiducialId());
                    }
                    Pose3D botPose = llResult.getBotpose_MT2();
                    if (botPose != null) {
                        //telemetry.addData("BotPose", botPose.toString());
                    }
                }
                telemetry.update();
            }
        }
    }

    private void shootleft() {
        resources.lgun.setPower(1);
        sleep(800);
        resources.llift.setPosition(0.625);
        sleep(300);
        resources.lgun.setPower(0);
        resources.llift.setPosition(1);
        sleep(500);
    }

    private void shootright() {
        resources.rgun.setPower(-1);
        sleep(800);
        resources.rlift.setPosition(0.3);
        sleep(300);
        resources.rlift.setPosition(0.005);
        resources.rgun.setPower(0);
        sleep(500);
    }

    private void rotate() {
        if (llift_status){
            resources.llift.setPosition(1);
        }
        if (rlift_status){
            resources.rlift.setPosition(0.005);
        }
        if (llift_status || rlift_status){
            rlift_status=false;
            llift_status=false;
            sleep(300);
        }
        if (carousel == 2) {
            carousel = 0;
            resources.lspin.setPower(-1);
            sleep(474);
            resources.lspin.setPower(0);
        } else if (carousel == 1) {
            carousel += 1;
            resources.rspin.setPower(-1);
            sleep(474);
            resources.rspin.setPower(0);
        } else if (carousel == 0) {
            carousel += 1;
            resources.fspin.setPower(-1);
            sleep(474);
            resources.fspin.setPower(0);
        }
    }

    private void shootgreen() {
        if (bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.GREEN)
            shootleft();
        else if (bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.GREEN)
            shootright();
    }

    private void shootpurple() {
        if (bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.PURPLE)
            shootleft();
        else if (bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.PURPLE)
            shootright();
    }
}