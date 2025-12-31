package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top202425;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top202425;

@Autonomous(name = "Auto_HangPrism_NotWork")
public class Auto_HangPrism_NT_NotWork extends LinearOpMode {
    private resources_top202425 resources;
    private controls_top202425 control;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //Servo servo = hardwareMap.servo.get("backclaw");

        resources = new resources_top202425(hardwareMap);

        control = new controls_top202425(resources.lsRight, resources.lsLeft, resources.lhs, resources.rhs, resources.intake
                , resources.blocker, resources.ril, resources.lil, resources.claw, resources.ra, resources.la, resources.backclaw);

        waitForStart();


        while (!isStopRequested() && !opModeIsActive()) {
            int position = 1;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = 1;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        //place teh first pixel
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        TrajectoryActionBuilder step1 = drive.actionBuilder(initialPose)
                .setReversed(true)
                .strafeTo(new Vector2d(-29.5, -10));

        //Go to wall to pickup another one
        Pose2d step1EndPose = new Pose2d(-29.5, -10, 0); // Assuming heading is 0
        TrajectoryActionBuilder step2 = drive.actionBuilder(step1EndPose)
                .strafeToConstantHeading(new Vector2d(-6,25))
                .turn(Math.toRadians(179))
                .lineToX(2)
                .waitSeconds(0.2);

        //move back to cage
        Pose2d step2EndPose = new Pose2d(3, 35, 180); // Assuming heading is 0
        TrajectoryActionBuilder step3 = drive.actionBuilder(step2EndPose)
                //.setReversed(true)
                .lineToX(0)
                .strafeToConstantHeading(new Vector2d(-26,-8))
                //.splineToLinearHeading(new Pose2d(-26,10, Math.toRadians(180)), Math.toRadians(90));
                .turn(Math.toRadians(128))
                .lineToX(-26.7);
        //move close to cage
        Pose2d step4EndPose = new Pose2d(-26.7, -8, 128); // Assuming heading is 0
        TrajectoryActionBuilder step4 = drive.actionBuilder(step4EndPose)
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-28.7,-8));

        //Push the first pixel back to wall
        Pose2d step5EndPose = new Pose2d(-28.7, -8, 128); // Assuming heading is 0
        TrajectoryActionBuilder step5 = drive.actionBuilder(step5EndPose)
                .splineToLinearHeading(new Pose2d(-18,25, Math.toRadians(92)), Math.toRadians(90));
                //.lineToX(-45);
                //.strafeTo(new Vector2d(-28.5, 35));

        Pose2d step6EndPose = new Pose2d(-22, -14, 0); // Assuming heading is 0
        TrajectoryActionBuilder step6 = drive.actionBuilder(step4EndPose)
                .lineToX(-26);

        Pose2d step7EndPose = new Pose2d(-26, -14, 0); // Assuming heading is 0
        TrajectoryActionBuilder step7 = drive.actionBuilder(step5EndPose)
                .lineToX(-22)
                .turn(Math.toRadians(-90))
                .lineToY(29)
                .turn(Math.toRadians(-90))
                .lineToX(-1.5 );
//                .lineToY(10)
//                .lineToX(60);

        Action MoveToCage1 = step1.build();
        //Action MoveToPush1 = step2.build();
        Action MoveToWall2 = step2.build();
        Action MoveBackToCage3 = step3.build();
        Action MoveCloseToCage3 = step4.build();
        Action MoveTo1stPixel = step5.build();
//        Action MoveToCage5 = step5.build();
//        Action PushPrism6 = step6.build();


        Thread hlsupThread = new Thread(new lsslideup());
        hlsupThread.start();

        //hang on the first pixel
        control.bclawclose();
        Actions.runBlocking(
                //new ParallelAction(
                new SequentialAction(
                        HlsStall(),
                        MoveToCage1,
                        LsReverse140(),
                        BclawOpen(),
                        MoveToWall2,

                        BclawClose(),
                        LsOn400()
                        //MoveBackToCage3
                        //LsOff()
                )
        );

//        Thread hlsupThread2 = new Thread(new lsslideuplonger());
//        hlsupThread.start();

        control.bclawclose();
        Actions.runBlocking(
                new SequentialAction(
                        HlsStall(),
                        MoveBackToCage3,
                        LsOnOff1000(),
                        MoveCloseToCage3,
                        LsReverse140(),
                        BclawOpen()
                        //MoveTo1stPixel
                )
        );

    }


    public class lsslideup implements Runnable {
        @Override
        //public

        public void run() {
            boolean stop = false;

            waitForStart();

            while (!Thread.interrupted() && opModeIsActive() && !stop) {
                // INTAKE LIFT STUFF HERE
                //up
                control.lson();
                sleep(1150);
                control.lsoff();
                stop = true;

            }
        }
    }

    public class lsslideuplonger implements Runnable {
        @Override
        //public

        public void run() {
            boolean stop = false;

            waitForStart();

            while (!Thread.interrupted() && opModeIsActive() && !stop) {
                // INTAKE LIFT STUFF HERE
                //up
                sleep(4000);
                control.lson();
                sleep(1000);
                //control.lsonlowpower();
                //sleep(1000);
                //control.lsoff();
                stop = true;

            }
        }
    }

    public class Hlsstall implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            control.hsstall();
            return false;
        }
    }
    public Action HlsStall() {
        return new Hlsstall();
    }

    public class Bclawopen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //control.bclawclose();
            sleep(50);
            control.bclawopen();
            sleep(50);
            return false;
        }
    }
    public Action BclawOpen() {
        return new Bclawopen();
    }

    public class Bclawclose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            control.bclawclose();
            sleep(300);
            return false;
        }
    }
    public Action BclawClose() {
        return new Bclawclose();
    }


    // within the Claw class
    public class Lson implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            control.lson();
            return false;
        }
    }
    public Action LsOn() {
        return new Lson();
    }

    // within the Claw class
    public class Lsonoff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            control.lson();
            sleep(1000);
            control.lsoff();
            return false;
        }
    }
    public Action LsOnOff1000() {
        return new Lsonoff();
    }

    public class Lsonoff400 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            control.lson();
            sleep(800);
            control.lsoff();
            return false;
        }
    }
    public Action LsOnOff400() {
        return new Lsonoff400();
    }

    public class Lson400 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            control.lson();
            sleep(400);
            control.lsoff();
            return false;
        }
    }
    public Action LsOn400() {
        return new Lson400();
    }

    public class Lsonlittle implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            control.lsonlowpower();
            sleep(400);
            return false;
        }
    }
    public Action LsOnlittle() {
        return new Lsonlittle();
    }

    public class Lsoff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            control.lsoff();
            return false;
        }
    }
    public Action LsOff() {
        return new Lsoff();
    }

    public class Lsreverse implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
//            control.lsoff();
//            sleep(200);
            control.lsreverseMiddle();
            sleep(120);
            return false;
        }
    }
    public Action LsReverse140() {
        return new Lsreverse();
    }
    public class Lsreverse400 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
               control.lsoff();
                sleep(200);
                control.lsreverseMiddle();
                sleep(300);
                return false;
            }
    }
        public Action LsReverse400() {
            return new Lsreverse400();



    }

}
