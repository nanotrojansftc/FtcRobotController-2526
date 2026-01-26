package org.firstinspires.ftc.teamcode.NanoTrojans.Tools; // Package declaration

// --- IMPORTS ---
import com.acmerobotics.dashboard.FtcDashboard; // Dashboard for debugging on PC
import com.acmerobotics.dashboard.telemetry.TelemetryPacket; // Packet to send data to Dashboard
import com.qualcomm.robotcore.eventloop.opmode.OpMode; // Standard OpMode class
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; // Annotation to show this on the driver station
import com.qualcomm.robotcore.hardware.DcMotor; // Motor hardware class
import com.qualcomm.robotcore.hardware.DcMotorSimple; // Motor direction helper
import com.qualcomm.robotcore.hardware.Servo; // Servo hardware class
import com.bylazar.configurables.annotations.Configurable; // Allows editing variables via Dashboard
import com.bylazar.telemetry.TelemetryManager; // Custom telemetry manager
import com.bylazar.telemetry.PanelsTelemetry; // Custom telemetry panel
import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // PedroPathing constants
import com.pedropathing.geometry.BezierLine; // Defines a straight line path
import com.pedropathing.geometry.BezierPoint; // Defines a specific point in space
import com.pedropathing.follower.Follower; // The main class that makes the robot drive
import com.pedropathing.paths.PathChain; // Connects multiple paths together
import com.pedropathing.geometry.Pose; // Represents X, Y, and Heading

// Custom library import
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.PedroLimelightRelocalizer;

@Autonomous(name = "Blue Far Auto Final", group = "Autonomous") // Registers the OpMode name
@Configurable // Enables dashboard configuration
public class PedroAutonomous extends OpMode {

    // 1. Systems & Tools
    private TelemetryManager panelsTelemetry; // Manages messages sent to the phone
    public Follower follower; // The "driver" object from PedroPathing
    private PedroLimelightRelocalizer relocalizer; // Tool to fix position using camera
    private Paths paths; // Holds all your path definitions (See bottom of file)

    // 2. Hardware Mechanisms
    private DcMotor intakeMotor; // The motor that sucks in game elements
    private Servo carouselServoMain, carouselServoLeft, carouselServoRight; // Servos (currently unused)

    // 3. Configuration & Logic
    private boolean isRedAlliance = true; // Set to true if on Red team, false for Blue
    private int targetTagID; // ID of the AprilTag to look for
    private long stateStartTime; // Tracks when the current action started
    private int pathState; // Tracks which step of auto we are in

    // --- STATE CONSTANTS (The steps of your routine) ---
    private static final int DRIVING_PATH_1 = 0; // Step: Drive to first basket
    private static final int WAIT_DELAY_AFTER_PATH_1 = 1; // Step: Wait 5 seconds
    private static final int DRIVING_PATH_2 = 2; // Step: Drive to setup for sample
    private static final int AIM_AT_TAG = 50; // Step: Use camera to aim
    private static final int DRIVING_PATH_3 = 3; // Step: Drive slowly to intake
    private static final int DRIVING_PATH_4 = 4; // Step: Return to basket
    private static final int WAIT_DELAY_AFTER_PATH_4 = 11; // Step: Wait 5 seconds
    private static final int DRIVING_PATH_5 = 5; // Step: Drive to setup for second sample
    private static final int DRIVING_PATH_6 = 6; // Step: Drive slowly to intake second sample
    private static final int DRIVING_PATH_7 = 7; // Step: Return to basket
    private static final int DONE = 99; // Step: Auto finished

    @Override
    public void init() {
        // Initialize the custom telemetry system
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // A. Setup Follower (The Robot Driver)
        follower = Constants.createFollower(hardwareMap); // Create follower from hardware map

        // --- IMPORTANT: STARTING POSITION ---
        // This sets where the robot THINKS it is when you press INIT.
        // X = 87.5 inches, Y = 8.0 inches, Facing 90 degrees (Forward/Left)
        follower.setStartingPose(new Pose(87.500, 8.000, Math.toRadians(90)));

        // B. Setup Relocalizer (Camera System)
        relocalizer = new PedroLimelightRelocalizer(follower, hardwareMap);

        // C. Setup Hardware (Intake Motor)
        intakeMotor = hardwareMap.get(DcMotor.class, "intake"); // Find motor named "intake"
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse direction if needed
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use encoders for consistent speed

        // Servos are currently commented out
        // carouselServoMain = hardwareMap.get(Servo.class, "mainServo");
        // carouselServoLeft = hardwareMap.get(Servo.class, "leftServo");
        // carouselServoRight = hardwareMap.get(Servo.class, "rightServo");

        // D. Configure Alliance Target (Which AprilTag to look for)
        if (isRedAlliance) {
            targetTagID = 24; // Red Alliance Tag ID
        } else {
            targetTagID = 20; // Blue Alliance Tag ID
        }

        // Initialize the paths defined at the bottom of the file
        paths = new Paths(follower);

        // Set the initial state to the first path
        setPathState(DRIVING_PATH_1);

        // Send initialized message to telemetry
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Ensure max power is 100% at start
        follower.setMaxPower(1.0);

        // Start following Path 1 immediately when play is pressed
        follower.followPath(paths.Path1, true);

        // Set state tracker to Path 1
        setPathState(DRIVING_PATH_1);
    }

    @Override
    public void loop() {
        // 1. Update Pedro Pathing (Calculates where to move next)
        follower.update();

        // 2. Dashboard Visualization (Draws robot on PC screen)
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().drawImage("/images/field.png", 0, 0, 144, 144);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // 3. Update State Machine (Checks logic for switching steps)
        autonomousPathUpdate();

        // 4. Telemetry (Prints debug info to phone)
        panelsTelemetry.debug("Path State", pathState); // Current step
        panelsTelemetry.debug("X", follower.getPose().getX()); // Current X Position
        panelsTelemetry.debug("Y", follower.getPose().getY()); // Current Y Position
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading())); // Current Angle
        panelsTelemetry.update(telemetry); // Push data to phone
    }

    // This is the "Brain" of the autonomous. It decides when to switch tasks.
    public void autonomousPathUpdate() {
        // Calculate how many seconds have passed since the current state started
        double timeElapsed = (System.currentTimeMillis() - stateStartTime) / 1000.0;

        switch (pathState) {
            case DRIVING_PATH_1:
                // Check if the robot has finished driving Path 1
                if(!follower.isBusy()) {
                    setPathState(WAIT_DELAY_AFTER_PATH_1); // Move to Wait State
                }
                break;

            case WAIT_DELAY_AFTER_PATH_1:
                // Wait until 3 seconds have passed
                if(timeElapsed > 3.0) {
                    follower.followPath(paths.Path2, true); // Start driving Path 2
                    setPathState(DRIVING_PATH_2); // Update State
                }
                break;

            case DRIVING_PATH_2:
                // Check if Path 2 is done
                if(!follower.isBusy()) {
                    setPathState(AIM_AT_TAG); // Move to Aiming State
                }
                break;

            case AIM_AT_TAG:
                // Ensure robot isn't moving before aiming
                if(!follower.isBusy()) {
                    // Get the horizontal offset (tx) from the Limelight camera
                    double tx = relocalizer.getOffsetFromTag(targetTagID);

                    // Safety: Aim only if time < 2s and we actually see the tag (-999 means no tag seen)
                    if (timeElapsed < 2.0 && tx != -999) {
                        // If error is small (less than 1.5 degrees), we are aimed correctly
                        if (Math.abs(tx) < 1.5) {
                            startPath3(); // Aimed! Go.
                        } else {
                            // --- TURNING LOGIC ---
                            Pose currentPose = follower.getPose(); // Get current position
                            double currentHeading = currentPose.getHeading(); // Get current angle
                            double errorRadians = Math.toRadians(tx); // Convert camera error to radians
                            double targetHeading = currentHeading - errorRadians; // Calculate new angle

                            // Create a "Hold Point" to turn in place
                            BezierPoint holdPoint = new BezierPoint(currentPose.getX(), currentPose.getY());
                            follower.holdPoint(holdPoint, targetHeading); // Command the turn
                        }
                    } else {
                        // If we timed out or can't see the tag, just go anyway
                        startPath3();
                    }
                }
                break;

            case DRIVING_PATH_3:
                // This state runs WHILE Path 3 is driving (The Slow Intake Path)
                if(!follower.isBusy()) {
                    // Path 3 is Done.

                    // --- STOP INTAKE & RESET SPEED ---
                    intakeMotor.setPower(0); // Turn off Intake
                    follower.setMaxPower(1.0); // Set speed back to 100% for return trip

                    follower.followPath(paths.Path4, true); // Start Path 4 (Return to basket)
                    setPathState(DRIVING_PATH_4); // Update State
                }
                break;

            case DRIVING_PATH_4:
                // Check if Path 4 is done
                if(!follower.isBusy()) {
                    setPathState(WAIT_DELAY_AFTER_PATH_4); // Move to Wait State
                }
                break;

            case WAIT_DELAY_AFTER_PATH_4:
                // Wait for 3 seconds
                if(timeElapsed > 3.0) {
                    follower.followPath(paths.Path5, true); // Start Path 5
                    setPathState(DRIVING_PATH_5); // Update State
                }
                break;

            case DRIVING_PATH_5:
                // Check if Path 5 is done
                if(!follower.isBusy()) {
                    // Path 5 Done. We are at setup position (104, 65).

                    // --- START INTAKE & SPEED REDUCTION (50%) ---
                    intakeMotor.setPower(1.0); // Turn on Intake
                    follower.setMaxPower(0.5); // Slow down to 50% for accuracy

                    follower.followPath(paths.Path6, true); // Start Path 6 (Intake drive)
                    setPathState(DRIVING_PATH_6); // Update State
                }
                break;

            case DRIVING_PATH_6:
                // This state runs WHILE Path 6 is driving
                if(!follower.isBusy()) {
                    // Path 6 Done.

                    // --- STOP INTAKE & RESET SPEED ---
                    intakeMotor.setPower(0); // Turn off Intake
                    follower.setMaxPower(1.0); // Speed back to 100%

                    follower.followPath(paths.Path7, true); // Start Path 7 (Return to basket)
                    setPathState(DRIVING_PATH_7); // Update State
                }
                break;

            case DRIVING_PATH_7:
                // Check if Path 7 is done
                if(!follower.isBusy()) {
                    setPathState(DONE); // Auto is finished
                }
                break;

            case DONE:
                // Do nothing, just stay here
                break;
        }
    }

    // Helper method to start Path 3 logic (Intake On + Slow Down)
    private void startPath3() {
        // --- START INTAKE & SPEED REDUCTION (50%) ---
        intakeMotor.setPower(1.0); // Start Intake motor
        follower.setMaxPower(0.5); // Set max speed to 50%

        follower.followPath(paths.Path3, true); // Start driving Path 3
        setPathState(DRIVING_PATH_3); // Update State
    }

    // Helper method to update the state and reset the timer
    public void setPathState(int pState) {
        pathState = pState;
        stateStartTime = System.currentTimeMillis(); // Reset timer
        panelsTelemetry.debug("State Change", "To " + pState); // Log change
    }

    // --- PATH DEFINITIONS (Modify coordinates here) ---
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {

            // PATH 1: Start -> First Basket Drop
            Path1 = follower.pathBuilder()
                    // Start at (87.5, 8.0) -> Go to (86, 19)
                    .addPath(new BezierLine(new Pose(87.500, 8.000), new Pose(86.75, 27.5)))
                    // Rotate from 90 degrees -> 65 degrees while driving
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))
                    .build();

            // PATH 2: Basket -> Setup Position for 1st Sample
            Path2 = follower.pathBuilder()
                    // Start at (85.3, 12.0) -> Go to (104, 43)
                    .addPath(new BezierLine(new Pose(85.381, 12.077), new Pose(104.000, 43.000)))
                    // Rotate from 65 degrees -> -1 degree (Facing Right/East)
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(-1))
                    .build();

            // PATH 3: Setup -> Pickup 1st Sample (INTAKE ON)
            Path3 = follower.pathBuilder()
                    // Start at (104, 43) -> Drive forward to (145, 43)
                    .addPath(new BezierLine(new Pose(104.000, 43.000), new Pose(145.000, 43.000)))
                    // Keep heading locked at -1 degree
                    .setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(-1))
                    .build();

            // PATH 4: Pickup -> Return to Basket
            Path4 = follower.pathBuilder()
                    // Start at (145, 43) -> Go back to Basket (86, 19)
                    .addPath(new BezierLine(new Pose(145.000, 43.000), new Pose(86, 19)))
                    // Rotate from -1 degree -> 65 degrees (To face basket)
                    .setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(65))
                    .build();

            // PATH 5: Basket -> Setup Position for 2nd Sample
            Path5 = follower.pathBuilder()
                    // Start at Basket -> Go to (104, 65)
                    .addPath(new BezierLine(new Pose(85.381, 12.077), new Pose(104.000, 65.000)))
                    // Rotate from 65 degrees -> -1 degree
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(-1))
                    .build();

            // PATH 6: Setup -> Pickup 2nd Sample (INTAKE ON)
            Path6 = follower.pathBuilder()
                    // Start at (104, 65) -> Drive forward to (145, 65)
                    .addPath(new BezierLine(new Pose(104.000, 65.000), new Pose(145.000, 65.000)))
                    // Keep heading locked at -1 degree
                    .setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(-1))
                    .build();

            // PATH 7: Pickup -> Return to Basket
            Path7 = follower.pathBuilder()
                    // Start at (145, 65) -> Go back to Basket (86, 19)
                    .addPath(new BezierLine(new Pose(145.000, 65.000), new Pose(86, 19)))
                    // Rotate from -1 degree -> 65 degrees
                    .setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(65))
                    .build();
        }
    }
}