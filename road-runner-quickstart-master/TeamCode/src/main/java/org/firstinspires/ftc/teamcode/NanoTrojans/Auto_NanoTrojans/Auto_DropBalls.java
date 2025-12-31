package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top;

@Autonomous(name = "Auto_DropBalls")
public class Auto_DropBalls extends LinearOpMode {

    private resources_top resources;
    private controls_top control;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //Servo servo = hardwareMap.servo.get("backclaw");
        resources = new resources_top(hardwareMap);
        control = new controls_top(resources.lgun, resources.rgun,  resources.intake, resources.llift, resources.rlift, resources.fspin, resources.rspin, resources.lspin);

        waitForStart();

        Actions.runBlocking(
               drive.actionBuilder(new Pose2d(0, 0, 0))
                       .lineToX(-10)
                       //.stopAndAdd(new ServoAction(resourcesTop.lil, 0))

                        //.strafeTo(new Vector2d(20, -20))
                        //.splineTo(new Vector2d(5, 10), 0)

                        .build());


            //shooting left ball
            control.shootleftStart();
            sleep(500);
            control.leftliftUp();
            sleep(1000);
            control.shootleftStop();
            control.leftliftDown();


    }


}
