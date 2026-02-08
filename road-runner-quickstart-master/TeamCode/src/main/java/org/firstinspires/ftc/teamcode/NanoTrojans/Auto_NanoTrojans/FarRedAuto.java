package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NanoTrojans.SubSytems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NanoTrojans.SubSytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FarRedAutoModular", group = "Competition")
public class FarRedAuto extends LinearOpMode {

    // --- SUBSYSTEMS ---
    private Follower follower;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private Limelight3A limelight;

    // --- ENUMS FOR MODULAR PARAMETERS ---
    public enum SubsystemAction {
        INTAKE_SCAN,    // Runs the automation with sensors
        SHOOTER_PREP,   // Pre-revs flywheels using Limelight
        SHOOTER_FIRE,   // Fires until bench is empty
        IDLE,           // Keeps motors at 0
        STOP_ALL        // Force stop everything
    }

    public enum PathTiming {
        BEGINNING,  // Call action once before path
        DURING,     // Run action continuously while moving
        END         // Run action once path is completed
    }

    @Override
    public void runOpMode() {
        // 1. INITIALIZE HARDWARE
        // Using Constants.createFollower as shown in your example
        follower = Constants.createFollower(hardwareMap);

        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // 2. DEFINE PATHS (Using PathChain and Geometry imports)
        // Note: We define starting poses for the lines based on the end of the previous path

        // Start Pose
        Pose startPose = new Pose(87.500, 8.000, Math.toRadians(90));
        follower.setPose(startPose);

        PathChain Path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(87.500, 8.000, Math.toRadians(90)), new Pose(86.75, 28, Math.toRadians(90))))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(64))
                .build();

        PathChain Path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(86.75, 28, Math.toRadians(64)), new Pose(104.000, 45.000, Math.toRadians(-1))))
                .setLinearHeadingInterpolation(Math.toRadians(64), Math.toRadians(-1))
                .build();

        PathChain Path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(104.000, 45.000, Math.toRadians(-1)), new Pose(145.000, 45.000, Math.toRadians(-1))))
                .setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(-1))
                .build();

        PathChain Path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(145.000, 45.000, Math.toRadians(-1)), new Pose(86.75, 28, Math.toRadians(64))))
                .setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(64))
                .build();
        PathChain Path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(86.75, 28, Math.toRadians(64)), new Pose(104.000, 65.000, Math.toRadians(-1))  ) )
                .setLinearHeadingInterpolation(Math.toRadians(64), Math.toRadians(-1))
                .build();
        PathChain Path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(104.000, 65.000, Math.toRadians(-1)), new Pose(145.000, 65.000, Math.toRadians(-1)) ) )
                .setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(-1))
                .build();

        PathChain Path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(145.000, 65.000, Math.toRadians(-1)), new Pose(86.75, 28, Math.toRadians(64))))
                .setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(64))
                .build();

        telemetry.addData("Status", "Initialized & Ready");
        telemetry.update();

        waitForStart();

        // 3. EXECUTE MODULAR ROUTINE

        performTask(Path1, SubsystemAction.SHOOTER_PREP, PathTiming.DURING);
        handleSubsystemTask(SubsystemAction.SHOOTER_FIRE);
        performTask(Path2, SubsystemAction.IDLE, PathTiming.DURING);
        performTask(Path3, SubsystemAction.INTAKE_SCAN, PathTiming.DURING);
        performTask(Path4, SubsystemAction.SHOOTER_PREP, PathTiming.DURING);
        handleSubsystemTask(SubsystemAction.SHOOTER_FIRE);
        performTask(Path5, SubsystemAction.IDLE, PathTiming.DURING);
        performTask(Path6, SubsystemAction.INTAKE_SCAN, PathTiming.DURING);
        performTask(Path7, SubsystemAction.SHOOTER_PREP, PathTiming.END);
        handleSubsystemTask(SubsystemAction.SHOOTER_FIRE);
    }

    /**
     * Modular Orchestrator: Pass PathChain, Action, and Timing.
     */
    private void performTask(PathChain path, SubsystemAction action, PathTiming timing) {

        // BEGINNING: Call once before path starts
        if (timing == PathTiming.BEGINNING) {
            handleSubsystemTask(action);
        }

        // Start following the path
        follower.followPath(path, true); // 'true' for holdEnd is usually safe

        // BLOCKING LOOP: Runs while the robot drives
        while (opModeIsActive() && follower.isBusy()) {
            follower.update(); // CRITICAL: Must update follower every loop

            // DURING: Constant updates while the robot moves
            if (timing == PathTiming.DURING) {
                handleSubsystemTask(action);
            }

            // Telemetry for debugging
            telemetry.addData("Pathing", "Busy");
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.update();
        }

        // END: Call once path is completed
        if (timing == PathTiming.END) {
            handleSubsystemTask(action);
        }
    }

    /**
     * Translates the Enum Action into the specific Subsystem calls.
     */
    private void handleSubsystemTask(SubsystemAction action) {
        switch (action) {
            case INTAKE_SCAN:
                intake.runIntakeAutomation(false);
                break;

            case SHOOTER_PREP:
                shooter.updateLimelightOnly(limelight.getLatestResult());

                break;

            case SHOOTER_FIRE:
                // Firing logic: Shoot until all bench sensors report empty
                // (Ensure isFullyLoaded/isBenchEmpty logic exists in IntakeSubsystem)
                while (opModeIsActive() && (intake.isBenchFull() || intake.artifactDetectedAtIntake())) {
                    shooter.updateLimelightOnly(limelight.getLatestResult());


                    // Optional: Add logic here to pulse your 'spindexer' or 'lifts'
                    // if they are not automatically handled by the ShooterSubsystem's update()

                    telemetry.addData("Status", "Firing Sequence Active");
                    telemetry.update();
                }
                shooter.stop();
                break;

            case STOP_ALL:
                intake.stopIntake();
                shooter.stop();
                break;

            case IDLE:
                // Do nothing
                break;
        }
    }
}