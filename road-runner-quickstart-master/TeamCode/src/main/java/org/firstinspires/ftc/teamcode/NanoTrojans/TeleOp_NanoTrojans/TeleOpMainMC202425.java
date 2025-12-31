package org.firstinspires.ftc.teamcode.NanoTrojans.TeleOp_NanoTrojans;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.Control_Base_NT;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top202425;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_base;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top202425;

@TeleOp(name ="TeleOpMainMC202425", group = "TeleOp")

public class TeleOpMainMC202425 extends LinearOpMode {

    private controls_top202425 control;
    private resources_top202425 resources;
    private resources_base resourcesbase;
    private Control_Base_NT driveControl;
    //private DriveControl_Base driveControl;
    BNO055IMU imu;
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



    @Override
    public void runOpMode() throws InterruptedException {

        resources = new resources_top202425(hardwareMap);
        resourcesbase = new resources_base(hardwareMap);

        telemetry.addLine("Initialized");
        telemetry.addLine("Claw Initial Position");
        telemetry.update();

        driveControl = new Control_Base_NT(hardwareMap);
        control = new controls_top202425(resources.lsRight, resources.lsLeft, resources.lhs, resources.rhs, resources.intake
                , resources.blocker, resources.ril, resources.lil, resources.claw, resources.ra, resources.la, resources.backclaw);

        waitForStart();

        //Thread baseControlThread = new Thread(new baseControl());
        // Thread hlsThread = new Thread(new hls());
        Thread lsControlThread = new Thread(new lsControl());
        Thread liftThread = new Thread(new lift());
        Thread armThread = new Thread(new arm());

        //Start 2  threads
        //baseControlThread.start();
        lsControlThread.start();
        armThread.start();
        liftThread.start();


        //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        //base control thread, let's use road runner's base control which has breaks
        while (!Thread.interrupted() && opModeIsActive())
        {
            driveControl.driveRobot(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

    }//end of runOpMode


//    private class lsControl implements Runnable {
//        boolean clawClosed = false;
//
//        @Override
//        public void run() {
//
//            waitForStart();
//            while (!Thread.interrupted() && opModeIsActive()) {
//                // LINEAR SLIDES STUFF - usually right stick
//                double lspower = gamepad2.left_stick_y;
//                resources.lsRight.setPower(lspower);
//                resources.lsLeft.setPower(-lspower);
//                // HORIZONTAL CONTROL STUFF - usually left stick
//                double hlspower = gamepad2.right_stick_y;
//                if(hlspower > 0 ) {
//                    canNotMoveUp = false;
//                }
//                else if(hlspower < 0)
//                {
//                    canNotMoveUp = true;
//                }
//
//                if(!canNotExtend)
//                {
//                    resources.rhs.setPower(0.5 * hlspower);
//                    resources.lhs.setPower(0.5 * -hlspower);
//                }
//            }//end of while
//        }//end of run
//    }//end of thread lscontrol

    private class lsControl implements Runnable {
        boolean clawClosed = false;

        @Override
        public void run() {
            waitForStart();
            while (!Thread.interrupted() && opModeIsActive()) {
                // LINEAR SLIDES STUFF - usually right stick
                double lspower = gamepad2.left_stick_y;
                resources.lsRight.setPower(lspower);
                resources.lsLeft.setPower(-lspower);

                // HORIZONTAL CONTROL STUFF - usually left stick
                double hlspower = gamepad2.right_stick_y;

                // Apply a deadzone for the joystick input to avoid small unintended movements
                double deadzone = 0.1; // This means values smaller than this will be ignored
                if (Math.abs(hlspower) < deadzone) {
                    hlspower = 0;
                }

                if (hlspower > 0) {
                    canNotMoveUp = false;
                } else if (hlspower < 0) {
                    canNotMoveUp = true;
                }

                if (!canNotExtend) {
                    // Decrease speed further by multiplying by a smaller value
                    double adjustedSpeed = 0.3 * hlspower; // Use 0.3 for slower speed (adjust as needed)
                    resources.rhs.setPower(adjustedSpeed);
                    resources.lhs.setPower(-adjustedSpeed);
                }
            } // end of while
        } // end of run
    } // end of thread lsControl



    public class arm implements Runnable{
        @Override
        public void run() {

            waitForStart();
            while (!Thread.interrupted() && opModeIsActive()) {
//              ARM STUFF

                if (gamepad2.dpad_down){
                    //resources.ra.setPosition(0.3);
                    //resources.la.setPosition(0.7);
                    control.armup();
                }
                //
                if (gamepad2.dpad_up){
//                    control.openclaw();
                    control.armdown();
                    control.hsretract();

                }
                // HANG STUFF
//                if (gamepad1.left_trigger>0 && (!canNotMoveUp)){
//                    control.hangeron();
//                    canNotExtend = true;
//                }
//                if (gamepad1.right_trigger>0){
//                    control.hangerreverse();
//                    canNotExtend = false;
//                }
//                else{
//                    control.hangeroff();
//                    //canNotExtend = false;
//                }
//                if(gamepad1.dpad_down && (!canNotMoveUp) ){
//                    control.hangeron();
//                    sleep(1600);
//                    control.hangeroff();
//                    canNotExtend = true;
//                }




                //INTAKE STUFF - same as jason
                //0.double intake = gamepad2.left_stick_x;
                double intake =  gamepad2.right_stick_x;
                resources.intake.setPower(intake);

            }
        }
    }
    //done with intake lift
    public class lift implements Runnable {
        @Override
        public void run() {

            waitForStart();

            while (!Thread.interrupted() && opModeIsActive()) {
                // INTAKE LIFT STUFF HERE
                //up
                if (gamepad2.dpad_left) {


                    control.intakeup();
                }
                if (gamepad2.dpad_right)/*down*/ {
                    control.intakedown();
                }
                // CLAW STUFF
                if (gamepad2.left_bumper) {
                    control.openclaw();
                    //control.openclaw();
                }
                //close
                if (gamepad2.right_bumper) {
                    control.closeclaw();


                }
                //open- jasons control is x
                if (gamepad2.left_trigger>0){
                    control.bclawopen();

                }
                //close- jasons control is y
                if (gamepad2.right_trigger>0){
                    control.bclawclose();
                }


            }
        }
    }

        //claw done






}//end of big class









