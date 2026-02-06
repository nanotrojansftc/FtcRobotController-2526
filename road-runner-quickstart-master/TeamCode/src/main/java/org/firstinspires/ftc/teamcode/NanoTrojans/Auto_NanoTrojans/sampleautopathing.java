package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.PedroLimelightRelocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;





@Autonomous(name = "FARLEAVEBLUE-MATCH3")
public class sampleautopathing extends OpMode {

    private Follower follower;
    private PedroLimelightRelocalizer relocalizer;
    private Timer pathTimer, opModeTimer;

    // Simplified State Machine (No Shooting)
    public enum PathState {
        DRIVE_TO_WAYPOINT,  // Drive from Start -> Shooting Spot
        WAIT_AT_WAYPOINT,   // Stop briefly so you can visually check accuracy
        DRIVE_TO_END,       // Drive from Shooting Spot -> Loading Zone
        DONE
    }

    private PathState pathState;

    // --- Poses ---
    private final Pose startPose = new Pose(88, 8.000, Math.toRadians(90));
    private final Pose waypointPose = new Pose(88, 28, Math.toRadians(90)); // Old "Shooting Pose"

    // --- Paths ---
    private PathChain path1_StartToWaypoint;

    public void buildPaths() {
        // Path 1: Start -> Waypoint
        path1_StartToWaypoint = follower.pathBuilder()
                .addPath(new BezierLine(startPose, waypointPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), waypointPose.getHeading())
                .build();

    }

    public void stateMachineUpdate() {
        switch (pathState) {
            case DRIVE_TO_WAYPOINT:
                // Command the path once, then switch state
                if (!follower.isBusy()) { // Safety check ensures we don't spam commands
                    follower.followPath(path1_StartToWaypoint, true);
                    setPathState(PathState.DRIVE_TO_END);
                }
                break;

            case DRIVE_TO_END:
                // Wait for the final path to finish
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                // Do nothing, just hold position
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        // 1. Setup Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        // 2. Setup Limelight Relocalizer

        // 3. Build Paths
        buildPaths();

        pathState = PathState.DRIVE_TO_WAYPOINT;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.DRIVE_TO_WAYPOINT);
    }

    @Override
    public void loop() {
        // Essential Updates
        follower.update();

        // Logic
        stateMachineUpdate();

        // Telemetry
        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}