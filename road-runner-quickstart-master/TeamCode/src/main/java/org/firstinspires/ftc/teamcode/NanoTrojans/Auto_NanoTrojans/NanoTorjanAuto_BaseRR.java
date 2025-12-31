/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_base;


/**
 * This class contains the Autonomous Mode program.
 */
@Config
@Autonomous(name = "NanoTorjanAuto_BaseRR")
public class NanoTorjanAuto_BaseRR extends LinearOpMode {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx rearLeftMotor;
    private DcMotorEx rearRightMotor;

    private resources_base resourceBase;

//    private FtcDashboard dash = FtcDashboard.getInstance();
//    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        //resourceBase = new resources_base_NanoTrojans(hardwareMap);

        DcMotor motor = hardwareMap.get(DcMotor.class, "leftFront");

        if (motor instanceof DcMotorEx) {
            telemetry.addData("Motor Type", "Supports DcMotorEx");
        } else {
            telemetry.addData("Motor Type", "Only supports DcMotor");
        }
        telemetry.update();

        sleep(2000);


        //Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        frontLeftMotor = drive.leftFront;
        frontRightMotor = drive.rightFront;
        rearLeftMotor = drive.leftBack;
        rearRightMotor = drive.rightBack;
//        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
//        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
//        rearLeftMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
//        rearRightMotor = hardwareMap.get(DcMotorEx.class, "rightBack");

        // 🛠 Ensure Motors Are Not Null
        if (frontLeftMotor == null || frontRightMotor == null || rearLeftMotor == null || rearRightMotor == null) {
            telemetry.addData("Error", "One or more motors are NULL! Check hardware map.");
            telemetry.update();
            return;  // Stop execution if motors are not found
        }


        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);



        while (!isStopRequested() && !opModeIsActive()) {
            int position = 1;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        waitForStart();


        if (isStopRequested()) return;



        telemetry.addData("Build Action", 2);
        telemetry.update();
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToXSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToX(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)

                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);

      Action trajectoryActionChosen = tab3.build();

        telemetry.addData("Run Action", 3);
        telemetry.update();

//        if (trajectoryActionChosen == null) {
//            telemetry.addData("Error", "Trajectory action is NULL!");
//            telemetry.update();
//            return;
//        } else {
//            telemetry.addData("Executing Action", "Starting...");
//            telemetry.update();
//        }
//        //drive.followTrajectorySequence(trajSeq);
//        Actions.runBlocking(
//                new SequentialAction(trajectoryActionChosen)
//        );
        telemetry.addData("Front Left Power", frontLeftMotor.getPower());
        telemetry.addData("Front Right Power", frontRightMotor.getPower());
        telemetry.addData("Rear Left Power", rearLeftMotor.getPower());
        telemetry.addData("Rear Right Power", rearRightMotor.getPower());
        telemetry.update();




    }

    private void setRunMode(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        rearLeftMotor.setMode(mode);
        rearRightMotor.setMode(mode);
    }

}
