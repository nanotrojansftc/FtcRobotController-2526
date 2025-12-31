package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top202425;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top202425;

@Autonomous(name = "AutonTest_RR")
public class AutonTest_RR extends LinearOpMode {
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


           Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));


            // vision here that outputs position
            int visionOutputPosition = 1;

            TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                    .lineToYSplineHeading(33, Math.toRadians(0))
                    .waitSeconds(2)
                    .setTangent(Math.toRadians(90))
                    .lineToY(48)
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
                    .lineToYSplineHeading(33, Math.toRadians(180))
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(46, 30))
                    .waitSeconds(3);
            Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                    .strafeTo(new Vector2d(48, 12))
                    .build();

            // actions that need to happen on init; for instance, a claw tightening.
            //Actions.runBlocking(Claw.CloseClaw());

            while (!isStopRequested() && !opModeIsActive()) {
                int position = visionOutputPosition;
                telemetry.addData("Position during Init", position);
                telemetry.update();
            }

            int startPosition = visionOutputPosition;
            telemetry.addData("Starting Position", startPosition);
            telemetry.update();
            waitForStart();

            if (isStopRequested()) return;

            Action trajectoryActionChosen;
            if (startPosition == 1) {
                trajectoryActionChosen = tab1.build();
            } else if (startPosition == 2) {
                trajectoryActionChosen = tab2.build();
            } else {
                trajectoryActionChosen = tab3.build();
            }

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActionChosen,
//                            lift.liftUp(),
//                            claw.openClaw(),
//                            lift.liftDown(),
                            trajectoryActionCloseOut
                    )
            );
        }

//        Actions.runBlocking(
//
//                        drive.actionBuilder(new Pose2d(0, 0, 0))
//                       .lineToX(-20)
//                       .waitSeconds(2)
//                       .lineToX(20)
//                       .waitSeconds(2)
//                       .strafeTo(new Vector2d(20, -20))
//                       .waitSeconds(2)
//                       .turn(Math.toRadians(90))
//                       .waitSeconds(2)
//                       .splineTo(new Vector2d(5, 10), Math.toRadians(90))
//                       .waitSeconds(2)
//
//                        .build());
//    }


//    public class Lift {
//        private DcMotorEx lift;
//        private DcMotorEx lift;
//
//        public Lift(HardwareMap hardwareMap) {
//            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
//            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            lift.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//    }
    public class Claw {

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                control.closeclaw();
                return false;
            }
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
//    public class Claw {
//        private Servo claw;
//
//        public Claw(HardwareMap hardwareMap) {
//            claw = hardwareMap.get(Servo.class, "claw");
//        }
//    }

//    public class ServoAction implements Action {
//        Servo servo;
//        double position;
//        ElapsedTime timer;
//
//        public ServoAction(Servo s, double p) {
//            this.servo = s;
//            this.position = p;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if (timer == null) {
//                timer = new ElapsedTime();
//
//                servo.setPosition(position);
//            }
//
//            return timer.seconds() < 3;
//
//        }
//    }
}
