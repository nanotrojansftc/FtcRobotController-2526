package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top202425;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top202425;

@Autonomous(name = "Auto_HighBasket_NT")
public class Auto_HighBasket_NT extends LinearOpMode {

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

        Actions.runBlocking(
               drive.actionBuilder(new Pose2d(0, 0, 0))
                       .lineToX(-10)
                       //.stopAndAdd(new ServoAction(resourcesTop.lil, 0))

                        //.strafeTo(new Vector2d(20, -20))
                        //.splineTo(new Vector2d(5, 10), 0)

                        .build());

        //sleep(500);
        //linear slide give power
        control.lson();
        //wait for slides to go up fully
        sleep(2000);
        // stop power
        control.lsoff();

       // arm up
        control.armup();
        //wait for arm to go up fully

        sleep(2000);

        // resources.lsRight.setPower(0);
        // resources.lsLeft.setPower(0);

        //open claw
        control.openclaw();
        sleep(1000);
        //arm down
        control.armdown();
        // linear slides down

        control.lsreverse();
        sleep(2000);
        control.lsoff();
        // give power to intake

        control.intakehalf();
        // put intake down
        control.intakedown();


    }




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
//
//    // claw class
//    public class Claw {
//        private Servo claw;
//
//        public Claw(HardwareMap hardwareMap) {
//            claw = hardwareMap.get(Servo.class, "claw");
//        }
//    }



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
