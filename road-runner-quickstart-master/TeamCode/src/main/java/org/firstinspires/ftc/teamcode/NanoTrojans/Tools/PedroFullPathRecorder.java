package org.firstinspires.ftc.teamcode.NanoTrojans.Tools;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TOOL: Pedro Path Point Recorder", group = "TOOL")
public class PedroFullPathRecorder extends OpMode {

    private Follower follower;

    private Pose startPose = null;
    private boolean lastA = false;
    private boolean lastB = false;
    private int pathIndex = 1;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Match your usual starting pose from autonomous
        follower.setStartingPose(new Pose(87.500, 8.000, Math.toRadians(90)));

        // Instructions that show on Driver Hub during INIT
        telemetry.addLine("=== Pedro Path Point Recorder ===");
        telemetry.addLine("Controller: PS4 as gamepad1 (Options + Cross to set User 1).");
        telemetry.addLine("");
        telemetry.addLine("How to record a path segment:");
        telemetry.addLine("1) Move robot (by hand or driving) to START pose.");
        telemetry.addLine("2) Press Cross  (gamepad1.a) to save START.");
        telemetry.addLine("3) Move robot to END pose.");
        telemetry.addLine("4) Press Circle (gamepad1.b) to save END & print PathN.");
        telemetry.addLine("");
        telemetry.addLine("Output appears below as Java code you can copy into Paths.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Keep Pedro localization updated
        follower.update();
        Pose current = follower.getPose();

        double x = current.getX();
        double y = current.getY();
        double headingRad = current.getHeading();
        double headingDeg = Math.toDegrees(headingRad);

        // Section: live pose
        telemetry.addLine("---- LIVE POSE ----");
        telemetry.addData("Current X (in)", x);
        telemetry.addData("Current Y (in)", y);
        telemetry.addData("Heading (deg)", headingDeg);

        // Section: live button state (to debug PS4 mapping)
        telemetry.addLine("---- BUTTONS (PS4) ----");
        telemetry.addData("A (Cross)", gamepad1.a);
        telemetry.addData("B (Circle)", gamepad1.b);

        // A: store START pose (Cross)
        boolean a = gamepad1.a;
        if (a && !lastA) {
            startPose = new Pose(x, y, headingRad);

            telemetry.addLine("=== START POSE SET ===");
            telemetry.addData("Start X", x);
            telemetry.addData("Start Y", y);
            telemetry.addData("Start Heading (deg)", headingDeg);
        }
        lastA = a;

        // B: store END pose & print full path snippet (Circle)
        boolean b = gamepad1.b;
        if (b && !lastB && startPose != null) {

            double sx = startPose.getX();
            double sy = startPose.getY();
            double sheadingRad = startPose.getHeading();
            double sheadingDeg = Math.toDegrees(sheadingRad);

            String header = String.format(
                    "Path%d = follower.pathBuilder()", pathIndex);
            String line = String.format(
                    "        .addPath(new BezierLine(new Pose(%.3f, %.3f), new Pose(%.3f, %.3f)))",
                    sx, sy, x, y
            );
            String headingLine = String.format(
                    "        .setLinearHeadingInterpolation(Math.toRadians(%.1f), Math.toRadians(%.1f))",
                    sheadingDeg, headingDeg
            );
            String footer = "        .build();";

            telemetry.addLine("=== PATH SNIPPET (Path" + pathIndex + ") ===");
            telemetry.addLine("// Start at (" +
                    String.format("%.3f, %.3f", sx, sy) +
                    ") -> Go to (" +
                    String.format("%.3f, %.3f", x, y) + ")");
            telemetry.addLine(header);
            telemetry.addLine(line);
            telemetry.addLine(headingLine);
            telemetry.addLine(footer);

            pathIndex++;
        }
        lastB = b;

        // Short reminder at bottom every loop
        telemetry.addLine("");
        telemetry.addLine("Reminder: Cross = set START, Circle = finalize Path" + (pathIndex) + ".");
        telemetry.update();
    }
}
