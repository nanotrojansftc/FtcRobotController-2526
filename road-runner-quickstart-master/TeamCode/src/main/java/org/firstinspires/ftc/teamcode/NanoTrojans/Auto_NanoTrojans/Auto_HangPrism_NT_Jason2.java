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

@Autonomous(name = "Auto_HangPrism_NT_Jason2")
public class Auto_HangPrism_NT_Jason2 extends LinearOpMode {
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


        Pose2d initialPose = new Pose2d(0, 0, 0);
        TrajectoryActionBuilder step1 = drive.actionBuilder(initialPose)
                .lineToX(-25)
                .strafeTo(new Vector2d(-25, -8));

        Pose2d step1EndPose = new Pose2d(-25, -8, 0); // Assuming heading is 0
        TrajectoryActionBuilder step2 = drive.actionBuilder(step1EndPose)
                .lineToX(-28.5);

        // move to wall to pick up second pixel
        Pose2d step2EndPose = new Pose2d(-28.5, -8, 0); // Assuming heading is 0
        TrajectoryActionBuilder step3 = drive.actionBuilder(step2EndPose)
                .lineToX(-20)
                .turn(Math.toRadians(-90))
                .lineToY(30)
                .turn(Math.toRadians(-92))
                .lineToX(2);

        //back to close to cage
        Pose2d step3EndPose = new Pose2d(2, 30, 0); // Assuming heading is 0
        TrajectoryActionBuilder step4 = drive.actionBuilder(step3EndPose)
                //.lineToX(-22)
                //.strafeTo(new Vector2d(-22, -14));
                .strafeToConstantHeading(new Vector2d(-22,-14))
                .turn(Math.toRadians(180));
                //.lineToY(-14);

        //litter step toward cage
        Pose2d step4EndPose = new Pose2d(-22, -14, 0); // Assuming heading is 0
        TrajectoryActionBuilder step5 = drive.actionBuilder(step4EndPose)
                     .lineToX(-31);

        //back to wall
        Pose2d step5EndPose = new Pose2d(-31, -14, 0); // Assuming heading is 0
        TrajectoryActionBuilder step6 = drive.actionBuilder(step5EndPose)
                .lineToX(-22)
                .turn(Math.toRadians(-90))
                .lineToY(29)
                .turn(Math.toRadians(-90))
                .lineToX(-1.5 );
//                .lineToY(10)
//                .lineToX(60);

        Action MoveToCage1 = step1.build();
        Action MoveToCage2 = step2.build();
        Action MoveToWall3 = step3.build();
        Action MoveBackToCage4 = step4.build();
        Action MoveToCage5 = step5.build();
        Action PushPrism6 = step6.build();

        control.bclawclose();
        Actions.runBlocking(
                new SequentialAction(
                        HlsStall(),
                        //BclawClose(),
                        MoveToCage1,
                        LsOnOff1000(),
                        MoveToCage2,
                        LsReverse140(),
                        BclawOpen(),
                        LsOff(),
                        //BclawClose(),
                        LsReverse140(),
                        MoveToWall3,

                        BclawClose(),
                        LsOnOff400(),
                        MoveBackToCage4,
                        //move ls up
                        LsOnOff1000(),
                        MoveToCage5,   //close to case
                        LsReverse140(),//move Ls down
                        BclawOpen(),     //open claw
                        LsOff(),
                        PushPrism6

                )
        );

        //control.bclawopen();
        control.bclawclose();

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
            //sleep(50);
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
            sleep(200);
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
            sleep(400);
            control.lsoff();
            return false;
        }
    }
    public Action LsOnOff400() {
        return new Lsonoff400();
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


            control.lsreverseMiddle();
            sleep(140);
            return false;
        }
    }
    public Action LsReverse140() {
        return new Lsreverse();


    }




}
