package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous
public class AutonomousDecodeSPTwoIdealRed extends LinearOpMode{
    @SuppressWarnings("unused")
    private resources_top resources;
    private controls_top control;

    private colorsensors bench;
    private int carousel;

    private CRServo fspin;
    private CRServo rspin;
    private CRServo lspin;

    private Limelight3A limelight3A;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private double lgunpower = -0.9;
    private double rgunpower = 0.9;

    public enum PathState{
        // START POSITION -> END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS,
        SHOOT_POS,
        LOAD_POS
    }
    PathState pathState;
    private final Pose sP1 = new Pose(88.000, 8.000, Math.toRadians(90));
//    public final Pose aprilTagScanning = new Pose(72.000, 96.000, Math.toRadians(90));
    private final Pose shootPose = new Pose(88.000, 88.000, Math.toRadians(42));
    //private final Pose reloadPose
    private PathChain driveStartShootPos, driveReloadPos;

    public void buildPath(){
        //put in coordinates for starting pos to end pos
        driveStartShootPos = follower.pathBuilder()
                .addPath(new BezierLine(sP1, shootPose))
                .setLinearHeadingInterpolation(sP1.getHeading(),shootPose.getHeading())
                .build();
        driveReloadPos= follower.pathBuilder()
                .addPath(new BezierLine( shootPose,sP1))
                .setLinearHeadingInterpolation(sP1.getHeading(),shootPose.getHeading())
                .build();
//        driveShootPos = follower.pathBuilder()
//                .addPath(new BezierLine(aprilTagScanning,shootPose))
//                .setLinearHeadingInterpolation(aprilTagScanning.getHeading(),shootPose.getHeading())
//                .build();
    }
    public void statePathUpdate(){
        switch (pathState) {
            case DRIVE_STARTPOS:
                follower.followPath(driveStartShootPos,true);
                setPathState(PathState.SHOOT_POS); // reset the timer & make new state at ths sametime
                break;
            case SHOOT_POS:
                // check is follower done its path
                if (!follower.isBusy() &&  pathTimer.getElapsedTimeSeconds()>3){
                    if (resources.apriltagvalue == 21) {
                        shootGPP();
                    } else if (resources.apriltagvalue == 22) {
                        shootPGP();
                    } else if (resources.apriltagvalue == 23) {
                        shootPPG();
                    }
                    follower.followPath(driveReloadPos, true);
                    setPathState(PathState.LOAD_POS);

                }
                break;
            case LOAD_POS:
                //follower.followPath(driveReloadPos,true);
                //setPathState(PathState.); // reset the timer & make new state at ths sametime
                break;

//            case SHOOT_POS:
//                // check is follower done its path
//                if (!follower.isBusy()){
//                    if (resources.apriltagvalue == 21) {
//                        dropBalls.shootGPP();
//                    } else if (resources.apriltagvalue == 22) {
//                        dropBalls.shootPGP();
//                    } else if (resources.apriltagvalue == 23) {
//                        dropBalls.shootPPG();
//                    }
//                    //telemetry.addLine("done path 2");
//                }
//                break;
            default:
                //telemetry.addLine("no state commanded");
                break;
        }
    }

    public void  setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    public Integer scanAprilTag() {
        LLResult result = limelight3A.getLatestResult();
        if (result == null || !result.isValid()) return null;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (id >= 21 && id <= 23) {
                return id;   // <-- return immediately
            }
        }
        return null;
    }
    private void shootGPP()
    {
        boolean rotated = false;
        if(( bench.detectByHue(bench.left,telemetry) == colorsensors.DetectedColor.GREEN
                && bench.detectByHue(bench.right,telemetry) == colorsensors.DetectedColor.PURPLE))
        {
            resources.lgun.setPower(lgunpower);
            resources.rgun.setPower(rgunpower);
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
            resources.rgun.setPower(rgunpower);
            resources.lgun.setPower(lgunpower);
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
            resources.lgun.setPower(lgunpower);
            resources.rgun.setPower(rgunpower);
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
            resources.rgun.setPower(rgunpower);
            resources.lgun.setPower(lgunpower);
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
            resources.lgun.setPower(lgunpower);
            resources.rgun.setPower(rgunpower);
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
            resources.rgun.setPower(rgunpower);
            resources.lgun.setPower(lgunpower);
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



    private void shootleft() {
        resources.lgun.setPower(lgunpower);
        sleep(1000);
        resources.llift.setPosition(0.625);
        sleep(1200);
        resources.lgun.setPower(0);
        resources.llift.setPosition(1);
        sleep(500);
    }

    private void shootright() {
        resources.rgun.setPower(rgunpower);
        sleep(1000);
        resources.rlift.setPosition(0.3);
        sleep(800);
        resources.rlift.setPosition(0.005);
        resources.rgun.setPower(0);
        sleep(500);
    }

    private boolean shootgreen() {
        if (bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.GREEN) {
            shootleft();
        } else if (bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.GREEN) {
            shootright();
        } else {
            return false;
        }
        return true;
    }

    private boolean shootpurple() {
        if (bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.PURPLE) {
            shootleft();
        } else if (bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.PURPLE) {
            shootright();
        } else {
            return false;
        }
        return true;
    }

    private void rotate() {
        if (carousel == 2) {
            carousel = 0;
            lspin.setPower(-1);
            sleep(474);
            lspin.setPower(0);
        } else if (carousel == 1) {
            carousel += 1;
            rspin.setPower(-1);
            sleep(474);
            rspin.setPower(0);
        } else if (carousel == 0) {
            carousel += 1;
            fspin.setPower(-1);
            sleep(474);
            fspin.setPower(0);
        }
    }

    private boolean rotateGreenIn() {
        rotate();
        sleep(500);
        if (!(bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.GREEN
                || bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.GREEN)) {
            rotate();
            sleep(500);
        }
        if (!(bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.GREEN
                || bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.GREEN)) {
            rotate();
            sleep(500);
        }
        return bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.GREEN
                || bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.GREEN;
    }

    private boolean rotatePurpleIn() {
        rotate();
        sleep(500);
        if (!(bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.PURPLE
                || bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.PURPLE)) {
            rotate();
            sleep(500);
        }
        if (!(bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.PURPLE
                || bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.PURPLE)) {
            rotate();
            sleep(500);
        }
        return bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.PURPLE
                || bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.PURPLE;
    }
    public void runOpMode() throws InterruptedException {
        pathState = PathState.DRIVE_STARTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        resources = new resources_top(hardwareMap);
        control = new controls_top(resources.lgun, resources.rgun, resources.intake, resources.llift, resources.rlift, resources.fspin, resources.rspin, resources.lspin);
        bench = new colorsensors();
        bench.init(hardwareMap);

        carousel = 0;
        fspin = hardwareMap.crservo.get("fspin");
        rspin = hardwareMap.crservo.get("rspin");
        lspin = hardwareMap.crservo.get("lspin");

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.setPollRateHz(100);
        limelight3A.start();
        telemetry.setMsTransmissionInterval(11);
        follower.setStartingPose(sP1);
        buildPath();

        waitForStart();
        follower.setPose(sP1);
        opModeTimer.resetTimer();
        setPathState(pathState);
        while (opModeIsActive()) {
            follower.update();
            Integer tag = scanAprilTag();
            if (tag != null) {
                resources.apriltagvalue = tag;
            }
            telemetry.addData("tag", resources.apriltagvalue);
            statePathUpdate();
            telemetry.update();

        }
    }

//    @Override
//    public void init(){
//        pathState = PathState.DRIVE_STARTPOS;
//        pathTimer = new Timer();
//        opModeTimer = new Timer();
//        // opModeTimer.resetTimer();
//        follower = Constants.createFollower(hardwareMap);
//        // Todo add in any other init mechanisms
//
//        buildPath();
//        follower.setPose(sP1);
//    }
//
//    public void start(){
//        opModeTimer.resetTimer();
//        setPathState(pathState);
//    }
//
//
//    @Override
//    public void loop(){
//        follower.update();
//        statePathUpdate();
//
//        telemetry.addData("path state", pathState.toString());
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());

}
