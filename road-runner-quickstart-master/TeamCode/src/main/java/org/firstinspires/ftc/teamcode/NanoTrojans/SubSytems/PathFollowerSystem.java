package org.firstinspires.ftc.teamcode.NanoTrojans.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // Added for reference

public class PathFollowerSystem {
//    private Follower follower;
//    private LinearOpMode opMode;
//    private FtcDashboard dashboard;
//
//    public enum TriggerTime { BEFORE, DURING, AFTER, NONE }
//    public enum RobotAction { INTAKE_ON, SHOOTER_PREP, SHOOTER_FIRE, IDLE }
//
//    public PathFollowerSystem(Follower follower, LinearOpMode opMode) {
//        this.follower = follower;
//        this.opMode = opMode;
//        this.dashboard = FtcDashboard.getInstance();
//    }
//
//    public void followPathWithLogic(
//            PathChain path,
//            RobotAction action,
//            TriggerTime time,
//            IntakeSubsystem intake,
//            ShooterSubsystem shooter,
//            int obeliskId)
//    {
//        if (time == TriggerTime.BEFORE) {
//            executeAction(action, intake, shooter, obeliskId);
//        }
//
//        follower.followPath(path, true);
//
//        while (opMode.opModeIsActive() && follower.isBusy()) {
//            follower.update();
//
//            // --- Dashboard Integration ---
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.fieldOverlay().drawImage("/images/field.png", 0, 0, 144, 144);
//            // Draw robot pose on the dashboard map
//            packet.put("X", follower.getPose().getX());
//            packet.put("Y", follower.getPose().getY());
//            packet.put("Heading", follower.getPose().getHeading());
//            dashboard.sendTelemetryPacket(packet);
//
//            if (time == TriggerTime.DURING) {
//                executeAction(action, intake, shooter, obeliskId);
//            }
//        }
//
//        if (time == TriggerTime.AFTER) {
//            executeAction(action, intake, shooter, obeliskId);
//        }
//    }
//
//    private void executeAction(RobotAction action, IntakeSubsystem intake, ShooterSubsystem shooter, int obeliskId) {
//        switch (action) {
//            case INTAKE_ON:
//                // Uses the "flawless" automation logic from Victor's TeleOp
//                intake.runIntakeAutomation(false);
//                break;
//            case SHOOTER_PREP:
//                // Simply call update() with no arguments
//                shooter.update();
//                break;
//            case SHOOTER_FIRE:
//                // Executes the full fire/reset/rotate sequence
//                shooter.startSequence();
//                shooter.update();
//                break;
//            case IDLE:
//                intake.stopIntake();
//                shooter.stopAll();
//                break;
//        }
//    }
}