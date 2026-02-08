package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

// --- IMPORTS ---
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // IMPORTANT IMPORT
import com.pedropathing.paths.PathConstraints;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NanoTrojans.SubSytems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NanoTrojans.SubSytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.NanoTrojans.Subsystems.InitialScanSystem;

@Autonomous(name = "4th-Far Blue Modular Auto", group = "4thMeet")
public class FarBlueModularAutonomous extends LinearOpMode {

    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private InitialScanSystem scanner;
    private Paths robotPaths;
    private Limelight3A limelight;

    public enum SubsystemTrigger {
        BEFORE_PATH,
        AFTER_PATH,
        DURING_PATH,
        NONE
    }

    // --- INNER CLASS FOR PATHS ---
// --- INNER CLASS FOR PATHS ---
    public static class Paths {
        public PathChain Path1, Path2/*, Path3, Path4, Path5, Path6, Path7*/;

        // 0.5x speed constraints for intake paths
        private final PathConstraints SLOW_CONSTRAINTS = new PathConstraints(
                0.5,   // half of 0.99
                50,    // half of 100
                1,     // keep same angular vel
                1      // keep same angular accel
        );

        public Paths(Follower follower) {

            // PATH 1: Start -> First Basket Drop
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.500, 8.000), new Pose(60, 13.7472)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                    .build();
            // PATH 2: Basket -> Setup Position for 1st Sample
            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60, 13.7472), new Pose(60, 30)))
                    .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(90))
                    .build();

            // PATH 3: Setup -> Pickup 1st Sample (INTAKE ON, 0.5x speed)
//            Path3 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(34, 43.000), new Pose(4, 43.000)))
//                    .setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(1))
//                    .setConstraints(SLOW_CONSTRAINTS)
//                    .build();
//
//            // PATH 4: Pickup -> Return to Basket
//            Path4 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(145.000, 43.000), new Pose(83.9253, 23.1056)))
//                    .setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(64.7242))
//                    .build();
//
//            // PATH 5: Basket -> Setup Position for 2nd Sample
//            Path5 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(83.9253, 23.1056), new Pose(104.000, 65.000)))
//                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(1))
//                    .build();
//
//            // PATH 6: Setup -> Pickup 2nd Sample (INTAKE ON, 0.5x speed)
//            Path6 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(104.000, 65.000), new Pose(140.000, 65.000)))
//                    .setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(1))
//                    .setConstraints(SLOW_CONSTRAINTS)
//                    .build();
//
//            // PATH 7: Pickup -> Return to Basket
//            Path7 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(145.000, 65.000), new Pose(83.9253, 23.1056)))
//                    .setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(64.7242))
//                    .build();
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        // --- INITIALIZATION ---

        // Use the Constants factory method instead of 'new Follower'
        follower = Constants.createFollower(hardwareMap);

        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        intake  = new IntakeSubsystem(hardwareMap);
        scanner = new InitialScanSystem(hardwareMap);

        // Limelight (for shooter aiming / velocity)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        scanner.initLimelight();

        // Initialize the paths inner class
        robotPaths = new Paths(follower);

        follower.setStartingPose(new Pose(56.5, 8.0, Math.toRadians(90)));

        // --- SCANNING LOOP ---
        while (!isStarted() && !isStopRequested()) {
            scanner.scanObelisk();
            scanner.scanStartingInventory();
            telemetry.addData("Obelisk ID", scanner.getObeliskId());
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        // --- SEQUENTIAL EXECUTION ---

        // Step 1: Drive to basket and shoot preload
        runStep(robotPaths.Path1, "SHOOT", SubsystemTrigger.AFTER_PATH);

        // Step 2: Setup 1st Sample
        runStep(robotPaths.Path2, null, SubsystemTrigger.NONE);

        // Step 3: Intake 1st & 2nd Sample (continuous until full)
//        runStep(robotPaths.Path3, "INTAKE", SubsystemTrigger.DURING_PATH);
//
//        // Step 4: Return and Shoot 1st Sample
//        runStep(robotPaths.Path4, "SHOOT", SubsystemTrigger.AFTER_PATH);
//
//        // Step 5: Setup 2nd Sample
//        runStep(robotPaths.Path5, null, SubsystemTrigger.NONE);
//
//        // Step 6: Intake 2nd/3rd Sample (if needed)
//        runStep(robotPaths.Path6, "INTAKE", SubsystemTrigger.DURING_PATH);
//
//        // Step 7: Return and Shoot 2nd Sample
//        runStep(robotPaths.Path7, "SHOOT", SubsystemTrigger.AFTER_PATH);
    }

    private void runStep(PathChain path, String action, SubsystemTrigger trigger) {
        if (!opModeIsActive()) return;

        if (trigger == SubsystemTrigger.BEFORE_PATH && action != null) {
            executeSubsystem(action);
        }

        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            // Run intake automation during intake paths (like in TestIntakeSystem loop)
            if (trigger == SubsystemTrigger.DURING_PATH && "INTAKE".equals(action)) {
                intake.runIntakeAutomation(false);
            }

            shooter.printDiagnostics(scanner.getObeliskId());
            telemetry.addData("Intake Raw", intake.getIntakeRawSum());
            telemetry.addData("Intake Detected", intake.artifactDetectedAtIntake());
            telemetry.addData("Left Bench Full", intake.isLeftBenchFull());
            telemetry.addData("Right Bench Full", intake.isRightBenchFull());
            telemetry.addData("Intook", intake.getArtifactsIntakenCount());
            telemetry.update();
        }

        follower.breakFollowing();

        // After the path, if we were intaking, keep intaking until fully loaded
        if (trigger == SubsystemTrigger.DURING_PATH && "INTAKE".equals(action)) {
            while (opModeIsActive()
                    && !intake.isFullyLoaded()
                    && intake.getArtifactsIntakenCount() < 3) {
                intake.runIntakeAutomation(false);

                shooter.printDiagnostics(scanner.getObeliskId());
                telemetry.addData("Intake Raw", intake.getIntakeRawSum());
                telemetry.addData("Intake Detected", intake.artifactDetectedAtIntake());
                telemetry.addData("Left Bench Full", intake.isLeftBenchFull());
                telemetry.addData("Right Bench Full", intake.isRightBenchFull());
                telemetry.addData("Intook", intake.getArtifactsIntakenCount());
                telemetry.update();
            }
            intake.stopIntake();
        }

        if (trigger == SubsystemTrigger.AFTER_PATH && action != null) {
            executeSubsystem(action);
        }
    }

    private void executeSubsystem(String type) {
        switch (type) {
            case "SHOOT":
                // 1) Spin up and aim until flywheel ready (same logic as TestShooterSubsystem)
                while (opModeIsActive() && !shooter.isFlywheelReady(0.05)) {
                    shooter.updateLimelightOnly(limelight.getLatestResult());
                    shooter.printDiagnostics(scanner.getObeliskId());
                    telemetry.addData("Phase", "Spinning up");
                    telemetry.update();
                }

                // 2) Shoot back pair (two artifacts)
                telemetry.addData("Phase", "Shooting back pair");
                telemetry.update();
                shooter.shootBackPair(this);

                // 3) Index once to bring front artifact into firing chamber
                telemetry.addData("Phase", "Indexing carousel");
                telemetry.update();
                shooter.indexOneCompartment(this);

                // 4) Shoot final artifact
                telemetry.addData("Phase", "Shooting last artifact");
                telemetry.update();
                shooter.shootSingle(this);

                // 5) Stop shooter
                shooter.stop();
                break;

            case "INTAKE":
                // not used in this flow, intake is handled in runStep
                intake.runIntakeAutomation(false);
                break;

            case "IDLE":
                shooter.idle();
                intake.stopIntake();
                break;
        }
    }
}
