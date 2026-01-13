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
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.Control_Base_NT;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors_2ndRobot;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top_2ndRobot;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_base;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top_2ndRobot;

import java.util.List;

@TeleOp(name ="TeleOpMainMC2ndRobot", group = "TeleOp")

public class TeleOpMainMC2ndRobot extends LinearOpMode {
    colorsensors_2ndRobot bench ;
    colorsensors_2ndRobot.DetectedColor left;
    colorsensors_2ndRobot.DetectedColor right;
    colorsensors_2ndRobot.DetectedColor back;
    float firsthue ;
    float secondhue;
    float thirdhue;

    private controls_top_2ndRobot control;
    private resources_top_2ndRobot resources;
    private resources_base resourcesbase;
    private Control_Base_NT driveControl;
    //private DriveControl_Base driveControl;

    //        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    //        parameters.loggingEnabled = true;
    //        parameters.loggingTag = "IMU";
    //        imu = hardwareMap.get(BNO055IMU.class, "imu");
    //        imu.initialize(parameters);


    private Limelight3A limelight3A;
    private IMU imu;
    //BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        bench = new colorsensors_2ndRobot();
        bench.init(hardwareMap);
        resources = new resources_top_2ndRobot(hardwareMap);
        resourcesbase = new resources_base(hardwareMap);

        driveControl = new Control_Base_NT(hardwareMap);
        control = new controls_top_2ndRobot(resources.shooter, resources.intake, resources.lift1, resources.lift2, resources.lift3);

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

        Thread intakeControlThread = new Thread(new intakeControl());
        Thread shootThread = new Thread(new shooting());
        Thread limelightThread = new Thread(new limelight());


        //Start 2  threads
        //baseControlThread.start();
        intakeControlThread.start();
        shootThread.start();
        limelightThread.start();

        //base control thread, let's use road runner's base control which has breaks
        while (!Thread.interrupted() && opModeIsActive())
        {
            driveControl.driveRobot(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
        }

    }//end of runOpMode



    private class intakeControl implements Runnable {
        boolean clawClosed = false;

        @Override
        public void run() {
            waitForStart();


            while (!Thread.interrupted() && opModeIsActive()) {


                firsthue = bench.getHueValue(bench.colorsensor1,telemetry);
                secondhue = bench.getHueValue(bench.colorsensor2,telemetry);
                thirdhue = bench.getHueValue(bench.colorsensor3,telemetry);

                telemetry.addData("First color", "%s, Hue=%.2f" , bench.detectByHue(firsthue, telemetry),firsthue);
                telemetry.addData("Second color", "%s,Hue=%.2f" ,bench.detectByHue(secondhue, telemetry),secondhue);
                telemetry.addData("Third color", "%s,Hue=%.2f" , bench.detectByHue(thirdhue, telemetry),thirdhue);
                telemetry.addLine("Detecting color");
                telemetry.update();

                double intake = -gamepad2.left_stick_y;
                resources.intake.setPower(intake);
//                if(bench.detectByHue(bench.back,telemetry) == colorsensors.DetectedColor.UNKNOWN) {
//                    // LINEAR SLIDES STUFF - usually right stick
//                    double intake = gamepad2.left_stick_y;
//                    resources.intake.setPower(intake);
//                }
           } // end of while
        } // end of run
    } // end of thread lsControl



    public class shooting implements Runnable{
        @Override
        public void run() {

            waitForStart();
            while (!Thread.interrupted() && opModeIsActive()) {
//              LIFT ARMS
                //game pad 2-  left trigger, bumper, right trigger, bumper

                if (gamepad2.right_bumper){
                    resources.lift3.setPosition(0.2);
                }
                if (gamepad2.right_trigger>0) {
                    resources.lift3.setPosition(1);
                }

                if (gamepad2.left_bumper){
                    resources.lift1.setPosition(1);
                }
                if (gamepad2.left_trigger>0) {
                    resources.lift1.setPosition(0.425);
                }

                if (gamepad2.dpad_up) {
                    resources.lift2.setPosition(1);
                }

                if (gamepad2.dpad_down) {
                    resources.lift2.setPosition(0.425);
                }
                // LEFT AND RIGHT GUNS
                //dpad -  left , right , down
                if (gamepad2.dpad_left){
                    resources.shooter.setPower(1);
                }
                if (gamepad2.dpad_right){
                    resources.shooter.setPower(0);
                }


                //game pad 2 - button X
                if (gamepad2.x){
                     //rotate();
                }

                if (gamepad2.y){
                    //rotate();
                }

                if (gamepad2.a) {
                    //if(left== colorsensors.DetectedColor.GREEN )
                    //{
                    //shootleft();
                    //shootgreen();
                    //}
                }

                if (gamepad2.b) {
                    //if(left== colorsensors.DetectedColor.GREEN )
                    //{
                    //shootright();
                    //shootpurple();
                    //}
                }

//                if(bench.detectByHue(bench.back,telemetry) == colorsensors.DetectedColor.UNKNOWN)
//                {
//                    // LINEAR SLIDES STUFF - usually right stick
//                    double intake = gamepad2.left_stick_y;
//                    resources.intake.setPower(intake);
//                }

                //this is for auto rotate when there is an empty space available, always move the empty
                // space to the front, this code can cause problem as well, you can also
                // disable this code and manually rotate
//                if(bench.detectByHue(bench.back,telemetry) != colorsensors.DetectedColor.UNKNOWN )
//                {
//                    sleep(1000);
//                    if(bench.detectByHue(bench.back,telemetry) == colorsensors.DetectedColor.UNKNOWN
//                            || bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.UNKNOWN
//                            || bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.UNKNOWN)
//                    {
//                        rotate();
//                        sleep(500);
//
//                    }
//                    //resources.intake.setPower(0);
//                }
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


    private void shootball()
    {
        resources.shooter.setPower(1);
        sleep(800);    //spin the ball for a while
        resources.lift1.setPosition(0.625);
        sleep(300);
        resources.shooter.setPower(0);
        resources.lift1.setPosition(1);
        sleep(500);
    }


    private void getGreen()
    {

    }

    private void getPurple()
    {

    }
}//end of big class









