package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top202425;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_base;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top202425;

@Autonomous(name = "First RoadeRunner Auton")
public class FirstRoadrunnerAuton extends LinearOpMode {
    private resources_top202425 resources;
    private controls_top202425 control;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Servo servo = hardwareMap.servo.get("backclaw");

        resources = new resources_top202425(hardwareMap);

        control = new controls_top202425(resources.lsRight, resources.lsLeft, resources.lhs, resources.rhs, resources.intake
                , resources.blocker, resources.ril, resources.lil, resources.claw, resources.ra, resources.la, resources.backclaw);

        waitForStart();

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
//        TrajectoryActionBuilder step1 = drive.actionBuilder(initialPose)
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-23, -22), Math.toRadians(0))
//                .lineToX(-28);
//        Action MoveToCage1 = step1.build();

        Pose2d step1EndPose = new Pose2d(0, 0, 0); // Assuming heading is 0
        TrajectoryActionBuilder step2 = drive.actionBuilder(step1EndPose)
                //.lineToXLinearHeading(20,Math.toRadians(90))
                //.splineToSplineHeading()
                .splineToSplineHeading(new Pose2d(12,32, Math.toRadians(180.5)), Math.toRadians(90));
                //.splineToLinearHeading(new Pose2d(12,32, Math.toRadians(90)), Math.toRadians(90))
                //.splineToLinearHeading(new Pose2d(-23,38,Math.toRadians(90)),Math.toRadians(90));
//                .strafeTo(new Vector2d(-23,42))
//                .turn(90);

                //.splineToSplineHeading(new Pose2d(-23,38, Math.toRadians(180)), Math.toRadians(0));
//                .strafeToConstantHeading(new Vector2d(-23,43))
//                .strafeToConstantHeading(new Vector2d(29,43));
                //.lineToXConstantHeading(68);42
        Action MoveToCage1 = step2.build();

        Pose2d step2EndPose = new Pose2d(0, 0, 0); // Assuming heading is 0
        TrajectoryActionBuilder step3 = drive.actionBuilder(step1EndPose)
                //.lineToXLinearHeading(20,Math.toRadians(90))
                //.splineToSplineHeading()
          //      .splineToSplineHeading(new Pose2d(12,32, Math.toRadians(180.5)), Math.toRadians(90));
        //.splineToLinearHeading(new Pose2d(24,43, Math.toRadians(180)), Math.toRadians(45));
                            .strafeToConstantHeading(new Vector2d(27,45))
        .turn(Math.toRadians(180))
                .lineToX(32);

//                .turn(Math.toRadians(90))
//                .turn(Math.toRadians(180))
//        .splineToLinearHeading(new Pose2d(36,45,Math.toRadians(180)),Math.toRadians(45));
//                .strafeTo(new Vector2d(-23,42))
//                .turn(90);

        //.splineToSplineHeading(new Pose2d(-23,38, Math.toRadians(180)), Math.toRadians(0));
//                .strafeToConstantHeading(new Vector2d(-23,43))
//                .strafeToConstantHeading(new Vector2d(29,43));
        //.lineToXConstantHeading(68);42
        Action MoveToWall = step3.build();

        Actions.runBlocking(
                new ParallelAction(
                        //MoveToCage1
                        MoveToWall
                )
        );
//        Actions.runBlocking(
//           //drive.actionBuilder(new Pose2d(33, -63, Math.toRadians(0)))
//                drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
//                        .splineTo(new Vector2d(20, 30), Math.toRadians(185))
//
//                        .build());
//        }

//         Actions.runBlocking(
//                 //drive.actionBuilder(new Pose2d(33, -63, Math.toRadians(0)))
//                 drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
//            .splineTo(new Vector2d(-20, -10), Math.toRadians(0))
//
//            .build());
//         }

//        Actions.runBlocking(
//    //drive.actionBuilder(new Pose2d(33, -63, Math.toRadians(0)))
//                drive.actionBuilder(new Pose2d(0, -0, Math.toRadians(0)))
//            .splineTo(new Vector2d(12, -24), Math.toRadians(-90))
//            .build());
//       }


//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(-10, -61, Math.toRadians(90)))
//                        .strafeTo(new Vector2d(-10, -33))
//                        .splineTo(new Vector2d(12, -24), Math.toRadians(90))
//
//            .build());
//    }

//    myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-10, -61, Math.toRadians(90)))
//
//            .strafeTo(new Vector2d(-10,-33.8),new TranslationalVelConstraint(18))
//            .setTangent(180)
//                  .splineToConstantHeading(new Vector2d(-48,-38), Math.toRadians(90))
    }
    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            control.closeclaw();
            return false;
        }
    }

    public class ArmUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            control.armup();
            return false;
        }
    }

    public class ArmDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            control.armdown();
            return false;
        }
    }

    // claw class
    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }
    }

    public class ServoAction implements Action {
        Servo servo;
        double position;
        ElapsedTime timer;

        public ServoAction(Servo s, double p) {
            this.servo = s;
            this.position = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();

                servo.setPosition(position);
            }

            return timer.seconds() < 3;

        }
    }
}
