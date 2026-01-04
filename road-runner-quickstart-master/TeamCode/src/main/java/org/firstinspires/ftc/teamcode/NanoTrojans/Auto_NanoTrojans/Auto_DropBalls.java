package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top;

import java.util.List;


@Autonomous(name = "Auto_DropBalls")
public class Auto_DropBalls extends LinearOpMode {

    private resources_top resources;
    private controls_top control;
    colorsensors bench ;
    private int carousel;

    CRServo fspin;
    CRServo rspin;
    CRServo lspin;

    private Limelight3A limelight3A;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //Servo servo = hardwareMap.servo.get("backclaw");
        resources = new resources_top(hardwareMap);
        control = new controls_top(resources.lgun, resources.rgun,  resources.intake, resources.llift, resources.rlift, resources.fspin, resources.rspin, resources.lspin);
        bench = new colorsensors();
        bench.init(hardwareMap);
        carousel = 0;

        fspin = hardwareMap.crservo.get("fspin");
        rspin= hardwareMap.crservo.get("rspin");
        lspin = hardwareMap.crservo.get("lspin");

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

        // IMPORTANT: Pipeline must be an APRILTAG pipeline
        limelight3A.pipelineSwitch(0);
        limelight3A.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)

        telemetry.setMsTransmissionInterval(11);
        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight3A.start();

        waitForStart();

//        Actions.runBlocking(
//               drive.actionBuilder(new Pose2d(0, 0, 0))
//                       .lineToX(-5)
//                       //.stopAndAdd(new ServoAction(resourcesTop.lil, 0))
//
//                        //.strafeTo(new Vector2d(20, -20))
//                        //.splineTo(new Vector2d(5, 10), 0)
//
//                        .build());

        LLResult result = limelight3A.getLatestResult();
        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducials =
                    result.getFiducialResults();

            telemetry.addData("AprilTags Detected", fiducials.size());

            for (LLResultTypes.FiducialResult tag : fiducials) {
                telemetry.addData("AprilTag ID", tag.getFiducialId());
                if( resources.apriltagvalue >=21 &&  resources.apriltagvalue <=23)
                     resources.apriltagvalue= tag.getFiducialId();
                else
                     resources.towervalue= tag.getFiducialId();

                //21 GPP    22 PGP   23 PPG
            }

            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
            telemetry.update();
        } else {
            telemetry.addData("Limelight", "No Targets");
            telemetry.update();
        }
        if(resources.apriltagvalue == 21)
            shootGPP();
        else if(resources.apriltagvalue == 22)
            shootPGP();
        else if(resources.apriltagvalue == 23)
            shootPPG();
        //rotate();

    }

    private void shootGPP()
    {
         boolean rotated = false;
        if(( bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.GREEN
             && bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.PURPLE))
        {
            resources.lgun.setPower(1);
            resources.rgun.setPower(-1);
            sleep(800);
            resources.llift.setPosition(0.625);
            sleep(600);
            resources.rlift.setPosition(0.3);
            sleep(500);
            resources.lgun.setPower(0);
            resources.llift.setPosition(1);
            resources.rlift.setPosition(0.005);
            //sleep(300);
            resources.rgun.setPower(0);

            //shoot third ball
            sleep(500);
            rotatePurpleIn();
            sleep(500);
            shootpurple();


        }
        else if ( bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.GREEN
            && bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.PURPLE)
        {
            resources.rgun.setPower(-1);
            resources.lgun.setPower(1);
            sleep(800);
            resources.rlift.setPosition(0.3);

            sleep(600);
            resources.llift.setPosition(0.625);
            sleep(500);
            resources.rlift.setPosition(0.005);
            //sleep(300);
            resources.rgun.setPower(0);
            resources.lgun.setPower(0);
            resources.llift.setPosition(1);

            sleep(500);
            rotatePurpleIn();
            sleep(500);
            shootpurple();
        }
        else
        {
            rotateGreenIn();

            if (shootgreen()) {//do nothing
            } else {
                rotateGreenIn();
                sleep(500);
                //do it again
                shootgreen();
            }
            sleep(500);
            ;
            if (shootpurple()) {//do nothing
            } else {   //do it again
                rotatePurpleIn();
                sleep(500);
                shootpurple();
            }
            ;
            sleep(500);


            if (shootpurple()) {//do nothing
            } else {   //do it again
                rotatePurpleIn();
                sleep(500);
                shootpurple();
            }
            ;

        }

    }

    private void shootPGP()
    {

        if(( bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.GREEN
                && bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.PURPLE))
        {
            resources.lgun.setPower(1);
            resources.rgun.setPower(-1);
            sleep(800);
            resources.rlift.setPosition(0.3);
            sleep(500);
            resources.llift.setPosition(0.625);
            sleep(600);

            resources.lgun.setPower(0);
            resources.llift.setPosition(1);
            resources.rlift.setPosition(0.005);
            //sleep(300);
            resources.rgun.setPower(0);

            //shoot third ball
            sleep(500);
            rotatePurpleIn();
            sleep(500);
            shootpurple();


        }
        else if ( bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.GREEN
                && bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.PURPLE)
        {
            resources.rgun.setPower(-1);
            resources.lgun.setPower(1);
            sleep(800);
            resources.llift.setPosition(0.625);
            sleep(500);
            resources.rlift.setPosition(0.3);
            sleep(600);
            resources.rlift.setPosition(0.005);
            //sleep(300);
            resources.rgun.setPower(0);
            resources.lgun.setPower(0);
            resources.llift.setPosition(1);

            sleep(500);
            rotatePurpleIn();
            sleep(500);
            shootpurple();
        }
        else
        {
            rotatePurpleIn();


            ;
            if (shootpurple()) {//do nothing
            } else {   //do it again
                rotatePurpleIn();
                sleep(500);
                shootpurple();
            }
            ;
            sleep(500);

            if (shootgreen()) {//do nothing
            } else {
                rotateGreenIn();
                sleep(500);
                //do it again
                shootgreen();
            }
            sleep(500);

            if (shootpurple()) {//do nothing
            } else {   //do it again
                rotatePurpleIn();
                sleep(500);
                shootpurple();
            }
            ;

        }

    }

    private void shootPPG()
    {

        if(( bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.PURPLE
                && bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.PURPLE))
        {
            resources.lgun.setPower(1);
            resources.rgun.setPower(-1);
            sleep(800);
            resources.rlift.setPosition(0.3);
            sleep(500);
            resources.llift.setPosition(0.625);
            sleep(600);

            resources.lgun.setPower(0);
            resources.llift.setPosition(1);
            resources.rlift.setPosition(0.005);
            //sleep(300);
            resources.rgun.setPower(0);

            //shoot third ball
            sleep(500);
            rotateGreenIn();
            sleep(500);
            shootgreen();


        }
        else if ( bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.PURPLE
                && bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.PURPLE)
        {
            resources.rgun.setPower(-1);
            resources.lgun.setPower(1);
            sleep(800);
            resources.llift.setPosition(0.625);
            sleep(500);
            resources.rlift.setPosition(0.3);
            sleep(600);
            resources.rlift.setPosition(0.005);
            //sleep(300);
            resources.rgun.setPower(0);
            resources.lgun.setPower(0);
            resources.llift.setPosition(1);

            sleep(500);
            rotateGreenIn();
            sleep(500);
            shootgreen();
        }
        else
        {
            rotatePurpleIn();
            if (shootpurple()) {//do nothing
            } else {   //do it again
                rotatePurpleIn();
                sleep(500);
                shootpurple();
            }
            ;
            sleep(500);


            if (shootpurple()) {//do nothing
            } else {   //do it again
                rotatePurpleIn();
                sleep(500);
                shootpurple();
            }
            sleep(500);

            if (shootgreen()) {//do nothing
            } else {
                rotateGreenIn();
                sleep(500);
                //do it again
                shootgreen();
            }


            ;

        }

    }

    private void shootleft()
    {
        resources.lgun.setPower(1);
        sleep(1000);
        resources.llift.setPosition(0.625);
        sleep(1200);
        resources.lgun.setPower(0);
        resources.llift.setPosition(1);
        sleep(500);
    }

    private void shootright()
    {
        resources.rgun.setPower(-1);
        sleep(1000);
        resources.rlift.setPosition(0.3);
        sleep(800);
        resources.rlift.setPosition(0.005);
        //sleep(300);
        resources.rgun.setPower(0);
        sleep(500);
    }

    private void rotate()
    {

        if (carousel ==2){
            carousel =0;
            lspin.setPower(-1);
            sleep(474);
            lspin.setPower(0);
        }
        else if (carousel ==1){
            carousel +=1;
            rspin.setPower(-1);
            sleep(474);
            rspin.setPower(0);
        }
        else if (carousel ==0){
            carousel +=1;
            fspin.setPower(-1);
            sleep(474);
            fspin.setPower(0);
        }
    }

    private boolean rotateGreenIn()
    {
        rotate();
        sleep(500);
        if ( !(bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.GREEN
                || bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.GREEN))
        {
            rotate();
            sleep(500);

        }
        if ( !(bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.GREEN
                || bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.GREEN))
        {
            rotate();
            sleep(500);
        }

        if (bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.GREEN
                || bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.GREEN)
            return true;
        else
            return false;

    }

    private boolean rotatePurpleIn()
    {
        rotate();
        sleep(500);
        if ( !(bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.PURPLE
                || bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.PURPLE))
        {
            rotate();
            sleep(500);
        }
        if ( !(bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.PURPLE
                || bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.PURPLE))
        {
            rotate();
            sleep(500);
        }

        if (bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.PURPLE
                || bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.PURPLE)
            return true;
        else
            return false;

    }

    private boolean shootgreen()
    {

//        telemetry.addData("shootgreen - left launcher: ", bench.detectByHue(bench.left,telemetry) );
//        telemetry.addData("shootgreen - right launcher: ", bench.detectByHue(bench.right,telemetry) );
        if(bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.GREEN)
            shootleft();
        else if(bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.GREEN)
            shootright();
        else   //rotate
        {
//            rotate();
//
//            sleep(600);
            return false;
        }
        return true;
        //shootleft();
    }

    private boolean shootpurple()
    {
        if(bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.PURPLE)
            shootleft();
        else if(bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.PURPLE)
            shootright();
        else   //rotate
        {
//            rotate();
//            sleep(600);
            return false;
        }
        return true;
    }
}
