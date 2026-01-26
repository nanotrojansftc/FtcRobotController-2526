package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.DropBalls;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous
public class AutonomousDecodeSPOneIdealRed extends LinearOpMode{
    @SuppressWarnings("unused")
    private resources_top resources;
    private controls_top control;
    private DropBalls dropBalls;

    private colorsensors bench;
    private int carousel;

    private CRServo fspin;
    private CRServo rspin;
    private CRServo lspin;

    private Limelight3A limelight3A;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        // START POSITION -> END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS,
        SCANNING,
        SHOOT_POS
    }
    PathState pathState;
    private final Pose sP1 = new Pose(88, 8, Math.toRadians(90));
    public final Pose aprilTagScanning = new Pose(72.000, 96.000, Math.toRadians(90));
    private final Pose shootPose = new Pose(89.000, 99.000, Math.toRadians(38));
    private PathChain driveStartScanPos, driveShootPos;

    public void buildPath(){
        //put in coordinates for starting pos to end pos
        driveStartScanPos = follower.pathBuilder()
                .addPath(new BezierLine(sP1, aprilTagScanning))
                .setLinearHeadingInterpolation(sP1.getHeading(),aprilTagScanning.getHeading())
                .build();
        driveShootPos = follower.pathBuilder()
                .addPath(new BezierLine(aprilTagScanning,shootPose))
                .setLinearHeadingInterpolation(aprilTagScanning.getHeading(),shootPose.getHeading())
                .build();
    }
    public void statePathUpdate(){
        switch (pathState) {
            case DRIVE_STARTPOS:
                follower.followPath(driveStartScanPos,true);
                setPathState(PathState.SCANNING); // reset the timer & make new state at ths sametime
                break;
            case SCANNING:
                // check is follower done its path
                if (!follower.isBusy() &&  pathTimer.getElapsedTimeSeconds()>3){
                    LLResult result = limelight3A.getLatestResult();
                    if (result != null && result.isValid()) {
                        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                        //telemetry.addData("AprilTags Detected", fiducials.size());

                        for (LLResultTypes.FiducialResult tag : fiducials) {
                            //telemetry.addData("AprilTag ID", tag.getFiducialId());
                            // Logic to determine which tag is detected
                            if (tag.getFiducialId() >= 21 && tag.getFiducialId() <= 23) {
                                resources.apriltagvalue = tag.getFiducialId();
                            } else {
                                resources.towervalue = tag.getFiducialId();
                            }
                        }
                        //telemetry.update();
                    } else {
                        //telemetry.addData("Limelight", "No Targets");
                        //telemetry.update();
                    }
                    follower.followPath(driveShootPos, true);
                    setPathState(PathState.SHOOT_POS);
                    //telemetry.addLine("done path 1");
                }
                break;
            case SHOOT_POS:
                // check is follower done its path
                if (!follower.isBusy()){
                    if (resources.apriltagvalue == 21) {
                        dropBalls.shootGPP();
                    } else if (resources.apriltagvalue == 22) {
                        dropBalls.shootPGP();
                    } else if (resources.apriltagvalue == 23) {
                        dropBalls.shootPPG();
                    }
                    //telemetry.addLine("done path 2");
                }
                break;
            default:
                //telemetry.addLine("no state commanded");
                break;
        }
    }

    public void  setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
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
        dropBalls = new DropBalls(this, resources, bench, fspin, rspin, lspin);
        telemetry.setMsTransmissionInterval(11);
        follower.setStartingPose(sP1);
        buildPath();

        waitForStart();
        follower.setPose(sP1);
        opModeTimer.resetTimer();
        setPathState(pathState);
        while (opModeIsActive()) {
            follower.update();
            statePathUpdate();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
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
