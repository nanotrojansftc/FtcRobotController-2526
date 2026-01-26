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





@Autonomous(name = "Test: Auto Pathing Only", group = "Test")
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
    private final Pose startPose = new Pose(88.000, 8.000, Math.toRadians(90));
    private final Pose waypointPose = new Pose(72, 96.00, Math.toRadians(90)); // Old "Shooting Pose"
    private final Pose endPose = new Pose(108.83, 84.41, Math.toRadians(180)); // Old "Load Pose"

    // --- Paths ---
    private PathChain path1_StartToWaypoint;
    private PathChain path2_WaypointToEnd;

    public void buildPaths() {
        // Path 1: Start -> Waypoint
        path1_StartToWaypoint = follower.pathBuilder()
                .addPath(new BezierLine(startPose, waypointPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), waypointPose.getHeading())
                .build();

        // Path 2: Waypoint -> End
        path2_WaypointToEnd = follower.pathBuilder()
                .addPath(new BezierLine(waypointPose, endPose))
                .setLinearHeadingInterpolation(waypointPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void stateMachineUpdate() {
        switch (pathState) {
            case DRIVE_TO_WAYPOINT:
                // Command the path once, then switch state
                if (!follower.isBusy()) { // Safety check ensures we don't spam commands
                    follower.followPath(path1_StartToWaypoint, true);
                    setPathState(PathState.WAIT_AT_WAYPOINT);
                }
                break;

            case WAIT_AT_WAYPOINT:
                // Wait until robot finishes path AND 2 seconds have passed
                // The 2 seconds lets you see if the Relocalizer "snaps" the robot into place
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    follower.followPath(path2_WaypointToEnd, true);
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
        relocalizer = new PedroLimelightRelocalizer(follower, hardwareMap);

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
        relocalizer.update(); // Keeps correcting X/Y while driving

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