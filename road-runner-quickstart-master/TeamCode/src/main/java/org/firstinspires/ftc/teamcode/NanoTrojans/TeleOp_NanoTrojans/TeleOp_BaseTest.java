//package org.firstinspires.ftc.teamcode;
package org.firstinspires.ftc.teamcode.NanoTrojans.TeleOp_NanoTrojans;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import teamcode.resources_NanoTrojans;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.Control_Base;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.Control_Base_NT;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.Control_Base_RR;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_base;
@TeleOp(name = "TeleOp_BaseTest", group = "TeleOp")


public class TeleOp_BaseTest extends LinearOpMode {

    private resources_base resourceBase;


    private Control_Base_NT driveControl_NT;
    private Control_Base_RR driveControl_RR;
    //private DriveControl_Base driveControl;
    @Override
    public void runOpMode()  throws InterruptedException {

        resourceBase = new resources_base(hardwareMap);

       //driveControl_NT = new Control_Base_NT(hardwareMap);

        driveControl_RR = new Control_Base_RR(hardwareMap);



        waitForStart();

        //Thread baseControlThread = new Thread(new baseControl_NT());
        Thread baseControlThread = new Thread(new baseControl_RR());

        //Start 2  threads
        baseControlThread.start();
        while (!isStopRequested())
        {

        }

    }

    // This is the thread class to control the base of the robot to move arround, this normally is
    // controlled by another person seperated from the base control person
    private class baseControl_NT implements Runnable {
        boolean droneLaunced = false;
        @Override
        public void run() {
            waitForStart();
            while (!Thread.interrupted() && opModeIsActive()) {
                // Motor control logic for motors 1 and 2
                //Call Robot base movement algorithem to drive the base
                driveControl_NT.driveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            }
        }
    }//end of class baseControl_NT


    // This is the thread class to control the base of the robot to move arround, this normally is
    // controlled by another person seperated from the base control person
    private class baseControl_RR implements Runnable {

        @Override
        public void run() {
            waitForStart();
            while (!Thread.interrupted() && opModeIsActive()) {
                // Motor control logic for motors 1 and 2
                //Call Robot base movement algorithem to drive the base
                driveControl_RR.driveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            }
        }
    }//end of class baseControl_NT



}//end of main class

