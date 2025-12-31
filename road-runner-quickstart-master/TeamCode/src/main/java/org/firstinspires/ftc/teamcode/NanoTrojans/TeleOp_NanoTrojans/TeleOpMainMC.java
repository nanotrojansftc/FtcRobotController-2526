package org.firstinspires.ftc.teamcode.NanoTrojans.TeleOp_NanoTrojans;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.Control_Base_NT;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_base;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@TeleOp(name ="TeleOpMainMC", group = "TeleOp")

public class TeleOpMainMC extends LinearOpMode {
    colorsensors bench ;
    colorsensors.DetectedColor left;
    colorsensors.rightcolor right;
    colorsensors.backcolor back;

    private controls_top control;
    private resources_top resources;
    private resources_base resourcesbase;
    private Control_Base_NT driveControl;
    //private DriveControl_Base driveControl;

    //        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    //        parameters.loggingEnabled = true;
    //        parameters.loggingTag = "IMU";
    //        imu = hardwareMap.get(BNO055IMU.class, "imu");
    //        imu.initialize(parameters);


    boolean lsStoped = false;
    public int rhspos;
    public int lhspos ;
    public boolean canNotMoveUp = true;
    public boolean canNotExtend = false;
    public int carousel = 0;


    private Limelight3A limelight3A;
    private IMU imu;
    //BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        bench = new colorsensors();
        bench.init(hardwareMap);
        resources = new resources_top(hardwareMap);
        resourcesbase = new resources_base(hardwareMap);

        driveControl = new Control_Base_NT(hardwareMap);
        control = new controls_top(resources.lgun, resources.rgun, resources.intake, resources.llift, resources.rlift, resources.fspin, resources.rspin, resources.lspin);

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight3A.start(); // This tells Limelight to start looking!
        // IMPORTANT: Pipeline must be an APRILTAG pipeline
        limelight3A.pipelineSwitch(0);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        waitForStart();

        Thread lsControlThread = new Thread(new lsControl());
        Thread liftThread = new Thread(new lift());
        Thread armThread = new Thread(new arm());
        Thread limelightThread = new Thread(new limelight());


        //Start 2  threads
        //baseControlThread.start();
        lsControlThread.start();
        armThread.start();
        liftThread.start();
        limelightThread.start();

        //base control thread, let's use road runner's base control which has breaks
        while (!Thread.interrupted() && opModeIsActive())
        {
            driveControl.driveRobot(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
        }

    }//end of runOpMode



    private class lsControl implements Runnable {
        boolean clawClosed = false;

        @Override
        public void run() {
            waitForStart();
            while (!Thread.interrupted() && opModeIsActive()) {
                // LINEAR SLIDES STUFF - usually right stick
                double intake = gamepad2.left_stick_y;
                resources.intake.setPower(intake);
                //resources.lsLeft.setPower(-lspower);

                // HORIZONTAL CONTROL STUFF - usually left stick
//                double intake =  gamepad2.right_stick_x;
//                resources.intake.setPower(intake);


            } // end of while
        } // end of run
    } // end of thread lsControl



    public class arm implements Runnable{
        @Override
        public void run() {

            waitForStart();
            while (!Thread.interrupted() && opModeIsActive()) {
//              LIFT ARMS
                //game pad 2-  left trigger, bumper, right trigger, bumper
                if (gamepad2.left_trigger>0) {
                    resources.llift.setPosition(1);
                }
                if (gamepad2.left_bumper){
                    resources.llift.setPosition(0.625);
                }
                if (gamepad2.right_trigger>0) {
                    resources.rlift.setPosition(0.005);
                }
                if (gamepad2.right_bumper){
                    resources.rlift.setPosition(0.3);
                }
                // LEFT AND RIGHT GUNS
                //dpad -  left , right , down
                if (gamepad2.dpad_left){
                    resources.lgun.setPower(1);
                }
                if (gamepad2.dpad_right){
                    resources.rgun.setPower(-1);
                }
                if (gamepad2.dpad_down) {
                    resources.lgun.setPower(0);
                    resources.rgun.setPower(0);
                }
                //game pad 2 - button X
                if (gamepad2.x){
                     rotate();
                }
            }
        }
    }

    //done with intake lift
    public class lift implements Runnable {
        @Override
        public void run() {

            waitForStart();

            while (!Thread.interrupted() && opModeIsActive()) {
                left = bench.getDetectedColor(telemetry);
                right = bench.getrightcolor(telemetry);
                back = bench.getbackcolor(telemetry);

                telemetry.addData("Left color", left);
                telemetry.addData("Right color", right);
                telemetry.addData("Back color", back);
                telemetry.update();

                if (gamepad2.a) {
                    //if(left== colorsensors.DetectedColor.GREEN )
                    //{
                    //shootleft();
                    shootgreen();
                    //}
                }

                if (gamepad2.b) {
                    //if(left== colorsensors.DetectedColor.GREEN )
                    //{
                    //shootright();
                    shootpurple();
                    //}
                }


            }

        }
    }

    public class limelight implements Runnable {
        @Override
        public void run() {

            waitForStart();

            // Update robot yaw for accurate botpose
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight3A.updateRobotOrientation(orientation.getYaw());

            LLResult llResult = limelight3A.getLatestResult();

            if (llResult != null && llResult.isValid()) {

                // ----- AprilTag (Fiducial) Detection -----
                List<LLResultTypes.FiducialResult> fiducials =
                        llResult.getFiducialResults();

                telemetry.addData("AprilTags Detected", fiducials.size());

                for (LLResultTypes.FiducialResult tag : fiducials) {
                    telemetry.addData("AprilTag ID", tag.getFiducialId());
                }

                // ----- Bot Pose from AprilTags -----
                Pose3D botPose = llResult.getBotpose_MT2();
                if (botPose != null) {
                    telemetry.addData("BotPose", botPose.toString());
                    telemetry.addData("Bot Yaw", botPose.getOrientation().getYaw());
                }

                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
            }
            telemetry.addData("AprilTags Detected", 2);
            telemetry.update();

        }
    }

    private void shootleft()
    {
        resources.lgun.setPower(1);
        sleep(500);
        resources.llift.setPosition(0.625);
        sleep(300);
        resources.lgun.setPower(0);
        resources.llift.setPosition(1);
    }

    private void shootright()
    {
        resources.rgun.setPower(-1);
        sleep(500);
        resources.rlift.setPosition(0.3);
        sleep(300);
        resources.rlift.setPosition(0.005);
        //sleep(300);
        resources.rgun.setPower(0);
    }

    private void rotate()
    {
        if (carousel ==2){
            carousel =0;
            resources.rspin.setPower(1);
            sleep(475);
            resources.rspin.setPower(0);
        }
        else if (carousel ==1){
            carousel +=1;
            resources.lspin.setPower(1);
            sleep(475);
            resources.lspin.setPower(0);
        }
        else if (carousel ==0){
            carousel +=1;
            resources.fspin.setPower(1);
            sleep(475);
            resources.fspin.setPower(0);
        }
    }

    private void shootgreen()
    {
        if(bench.getDetectedColor(telemetry) == colorsensors.DetectedColor.GREEN)
            shootleft();
        else if(bench.getrightcolor(telemetry) == colorsensors.rightcolor.GREEN)
            shootright();
        else   //rotate
        {
            //rotate();
            //sleep(200);
        }
    }

    private void shootpurple()
    {
        if(bench.getDetectedColor(telemetry) == colorsensors.DetectedColor.PURPLE)
            shootleft();
        else if(bench.getrightcolor(telemetry) == colorsensors.rightcolor.PURPLE)
            shootright();
        else   //rotate
        {
            //rotate();
            //sleep(200);
        }
    }
}//end of big class









