package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top202425;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top202425;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_base;

@Autonomous(name = "Encoder Auto Movement", group = "Linear Opmode")
public class EncoderAutoMovement extends LinearOpMode {

    // Declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // Constants for motion calculations
    static final double COUNTS_PER_MOTOR_REV = 537.6;  // REV HD Hex Motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;   // No gear reduction
    static final double WHEEL_DIAMETER_INCHES = 4.0;  // Wheel diameter in inches
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * Math.PI);

    private resources_top202425 resources;
    private controls_top202425 control;
    private resources_base resourcesbase;

    private boolean lockhls = true;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders and set to RUN_USING_ENCODER mode
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resources = new resources_top202425(hardwareMap);
        resourcesbase = new resources_base(hardwareMap);
        control = new controls_top202425(resources.lsRight, resources.lsLeft, resources.lhs, resources.rhs, resources.intake
                , resources.blocker, resources.ril, resources.lil, resources.claw, resources.ra, resources.la, resources.backclaw);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            //lock the hls
            Thread hlsControlLockThread = new Thread(new hlsLock());
            hlsControlLockThread.start();


            //score
            control.bclawclose();
            Pose2d initialPose = new Pose2d(0, 0, 0);
            TrajectoryActionBuilder step1 = drive.actionBuilder(initialPose)
                    //.lineToX(-18)
                    .strafeTo(new Vector2d(-18, -18));
            Action putFirstPixelTobasket = step1.build();

//            Pose2d step2Pose = new Pose2d(-18, -18, 0);
//            TrajectoryActionBuilder step2 = drive.actionBuilder(initialPose)
//                    //.lineToX(-18)
//                    .strafeTo(new Vector2d(-18, -18));
//            Action putFirstPixelTobasket = step1.build();

            Actions.runBlocking(
                    new SequentialAction(
                            putFirstPixelTobasket
                    )
            );


            //strafe(0.75, 5);
            //drive(0.75, 5);
            control.lson();
            sleep(500);
            drive(0.75,6.4);

            control.lsoff();
            sleep(400);
            control.lsreverse();
            sleep(150);

            control.lsoff();
            sleep(50);
            control.bclawopen();
            control.lsoff();
            sleep(1000);

//            drive(0.75,-5);
//            rightTurn(1,34);
//
//
//            strafe(0.75,35);
////            rightTurn(1,5);
//            control.lsreverse();
//            sleep(500);
//            control.lsoff();
//
//
//
//            drive(0.5,-20);
////            rightTurn(1,5);
//
//
//            strafe(0.5,7);
//            drive(0.5,30);
//            sleep(1000);
//
//            drive(0.5,-10);
//            sleep(1000);
//            drive(0.5,10);
//            control.bclawclose();
//
//
//            requestOpModeStop();
//



        }



    }

    private class hlsLock implements Runnable {
        boolean clawClosed = false;
        //double hspower1 , hspower2;

        @Override
        public void run() {
            boolean lock = false;

            waitForStart();
            while (!Thread.interrupted() && opModeIsActive() && lockhls)
            {
                //prevent hls to move out
                resources.rhs.setPower(0.5);
                resources.lhs.setPower(-0.5);
                //blocker up
                resources.blocker.setPosition(0.58);
            }//end of while
        }//end of run
    }//end of thread horizontal linear slide control
    /**
     * Drives the robot forward or backward.
     *
     * @param speed Speed of motion (positive for forward, negative for backward)
     * @param distanceInInches Distance to move in inches
     */
    public void drive(double speed, double distanceInInches) {
        int targetPosition = (int) (distanceInInches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + targetPosition);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + targetPosition);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + targetPosition);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + targetPosition);

        // Set to RUN_TO_POSITION mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        // Wait until motion is complete
        while (opModeIsActive() &&
                (leftFront.isBusy() && rightFront.isBusy() &&
                        leftBack.isBusy() && rightBack.isBusy())) {
            telemetry.addData("Path", "Driving to %7d", targetPosition);
            telemetry.update();
        }

        // Stop all motion
        stopMotors();
    }

    /**
     * Strafes the robot to the left or right.
     *
     * @param speed Speed of motion (positive for right, negative for left)
     * @param distanceInInches Distance to strafe in inches
     */
    public void strafe(double speed, double distanceInInches) {
        int targetPosition = (int) (distanceInInches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() - targetPosition);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() - targetPosition);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + targetPosition);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + targetPosition);

        // Set to RUN_TO_POSITION mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        // Wait until motion is complete
        while (opModeIsActive() &&
                (leftFront.isBusy() && rightFront.isBusy() &&
                        leftBack.isBusy() && rightBack.isBusy())) {
            telemetry.addData("Path", "Strafing to %7d", targetPosition);
            telemetry.update();
        }

        // Stop all motion
        stopMotors();
    }

    public void rightTurn(double speed, double distanceInInches) {
        int targetPosition = (int) (distanceInInches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() - targetPosition);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + targetPosition);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() - targetPosition);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + targetPosition);

        // Set to RUN_TO_POSITION mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        // Wait until motion is complete
        while (opModeIsActive() &&
                (leftFront.isBusy() && rightFront.isBusy() &&
                        leftBack.isBusy() && rightBack.isBusy())) {
            telemetry.addData("Path", "Strafing to %7d", targetPosition);
            telemetry.update();
        }

        // Stop all motion
        stopMotors();
    }

    public void leftTurn(double speed, double distanceInInches) {
        int targetPosition = (int) (distanceInInches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + targetPosition);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() - targetPosition);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + targetPosition);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() - targetPosition);

        // Set to RUN_TO_POSITION mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        // Wait until motion is complete
        while (opModeIsActive() &&
                (leftFront.isBusy() && rightFront.isBusy() &&
                        leftBack.isBusy() && rightBack.isBusy())) {
            telemetry.addData("Path", "Strafing to %7d", targetPosition);
            telemetry.update();
        }

        // Stop all motion
        stopMotors();
    }

    /**
     * Stops all motors.
     */
    private void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
