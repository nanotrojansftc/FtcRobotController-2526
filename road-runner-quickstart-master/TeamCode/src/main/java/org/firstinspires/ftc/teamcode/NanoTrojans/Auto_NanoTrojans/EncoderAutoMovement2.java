package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top202425;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top202425;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_base;


@Autonomous(name = "Encoder Auto Movement2", group = "Linear Opmode")
public class EncoderAutoMovement2 extends LinearOpMode {

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
                , resources.blocker, resources.ril, resources.lil, resources.claw, resources.ra, resources.la,  resources.backclaw);

        waitForStart();
        while (opModeIsActive() && !isStopRequested())

        {
            //close claw
            control.closeclaw();

            //retract horizontal linear slides
//            resources.rhs.setPower(-0.5);
//            resources.lhs.setPower(0.5);
            //blocker up
            control.intakeup();
            //resources.intake.setPower(1);
//
            //resources.claw.setPosition(0.6);
//          drive forward and strafe to basket
            drive(0.75,-7);

            strafe(0.75,-23);
            strafe(0.4,-8);
//            resources.rhs.setPower(1);
//            resources.lhs.setPower(-1);
            sleep(500);
//          linear slide give power
            control.lson();
            //wait for slides to go up fully


            sleep(2000);
            // stop power
            control.lsoff();

            //try to stall linear slides
            control.lsstall();


//          arm up
            control.armup();
            //wait for arm to go up fully

            sleep(2000);

//            resources.lsRight.setPower(0);
//            resources.lsLeft.setPower(0);

            //open claw
            control.openclaw();
            sleep(1000);
//            control.hsextend();
            //arm down
            control.armdown();
            // linear slides down

            control.lsreverse();
            sleep(2000);
            control.lsoff();
//            control.hsretract();
            // give power to intake

            control.intakehalf();
            // put intake down
            control.intakedown();
            //blocker up
            // strafe for adjust
            strafe(0.75,11);
            drive(0.5,-6.5);

            //push horizontal linear slides forward
            control.hsextend();
//            sleep(500);
//            resources.rhs.setPower(0);
//            resources.lhs.setPower(0);
//            resources.rhs.setPower(-0.5);
//            resources.lhs.setPower(0.5);


            //drive(0.5,-20);
            sleep(1500);

            // stop intake

            // retract horizontal linear slides again

            control.hsretract();

            // wait a bit
            sleep(1000);


            //put intake up
            control.intakeup();

            // lower blocker

            control.intakeoff();
            sleep(1000);
            control.closeclaw();
            control.hsstall();
            // move back towards the basket

           // drive(1,25);
            strafe(1, -11);
            drive(0.75,8);
            strafe(0.3, -7);


            //repeat scoring





            sleep(500);
//          linear slide give power
            control.lson();
            //wait for slides to go up fully


            sleep(2000);
            // stop power
            control.lsoff();

            //try to stall linear slides
            control.lsstall();


//          arm up
            control.armup();
            //wait for arm to go up fully

            sleep(2000);

//            resources.lsRight.setPower(0);
//            resources.lsLeft.setPower(0);

            //open claw
            control.openclaw();
            sleep(500);
            //arm down
            control.armdown();
            // linear slides down

            control.lsreverse();
            sleep(2000);
            control.lsoff();

//            control.hangeron();
//            strafe(1,10);
//            drive(1,-30);
//            leftTurn(1,25);
//            drive(0.75,30);
////            sleep(1000);
//            control.hangeroff();

//
            isStopRequested();
            requestOpModeStop();








        }





        // Drive forward 5 feet (60 inches)

    }

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
