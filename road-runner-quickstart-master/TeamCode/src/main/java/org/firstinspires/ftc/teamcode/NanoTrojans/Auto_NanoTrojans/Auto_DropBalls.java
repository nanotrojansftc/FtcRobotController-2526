package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

// --- FIXED PEDRO PATHING IMPORTS ---
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top;

import java.util.List;

@Autonomous(name = "Auto_DropBalls")
public class Auto_DropBalls extends LinearOpMode {

    private resources_top resources;
    private controls_top control;
    private colorsensors bench;
    private int carousel;

    private CRServo fspin;
    private CRServo rspin;
    private CRServo lspin;

    private Limelight3A limelight3A;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- FIXED INITIALIZATION ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

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

        waitForStart();

        LLResult result = limelight3A.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            telemetry.addData("AprilTags Detected", fiducials.size());

            for (LLResultTypes.FiducialResult tag : fiducials) {
                telemetry.addData("AprilTag ID", tag.getFiducialId());
                // Logic to determine which tag is detected
                if (tag.getFiducialId() >= 21 && tag.getFiducialId() <= 23) {
                    resources.apriltagvalue = tag.getFiducialId();
                } else {
                    resources.towervalue = tag.getFiducialId();
                }
            }
            telemetry.update();
        } else {
            telemetry.addData("Limelight", "No Targets");
            telemetry.update();
        }

        follower.update();

        if (resources.apriltagvalue == 21) {
            shootGPP();
        } else if (resources.apriltagvalue == 22) {
            shootPGP();
        } else if (resources.apriltagvalue == 23) {
            shootPPG();
        }
    }

    private void shootGPP() {
        if ((bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.GREEN
                && bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.PURPLE)) {
            performShotSequence(true, true);
            sleep(500);
            rotatePurpleIn();
            sleep(500);
            shootpurple();
        } else if (bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.GREEN
                && bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.PURPLE) {
            performShotSequence(false, true);
            sleep(500);
            rotatePurpleIn();
            sleep(500);
            shootpurple();
        } else {
            rotateGreenIn();
            if (!shootgreen()) {
                rotateGreenIn();
                sleep(500);
                shootgreen();
            }
            sleep(500);
            if (!shootpurple()) {
                rotatePurpleIn();
                sleep(500);
                shootpurple();
            }
            sleep(500);
            if (!shootpurple()) {
                rotatePurpleIn();
                sleep(500);
                shootpurple();
            }
        }
    }

    private void shootPGP() {
        if ((bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.GREEN
                && bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.PURPLE)) {
            performShotSequence(true, false);
            sleep(500);
            rotatePurpleIn();
            sleep(500);
            shootpurple();
        } else if (bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.GREEN
                && bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.PURPLE) {
            performShotSequence(false, false);
            sleep(500);
            rotatePurpleIn();
            sleep(500);
            shootpurple();
        } else {
            rotatePurpleIn();
            if (!shootpurple()) {
                rotatePurpleIn();
                sleep(500);
                shootpurple();
            }
            sleep(500);
            if (!shootgreen()) {
                rotateGreenIn();
                sleep(500);
                shootgreen();
            }
            sleep(500);
            if (!shootpurple()) {
                rotatePurpleIn();
                sleep(500);
                shootpurple();
            }
        }
    }

    private void shootPPG() {
        if ((bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.PURPLE
                && bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.PURPLE)) {
            performShotSequence(true, false);
            sleep(500);
            rotateGreenIn();
            sleep(500);
            shootgreen();
        } else if (bench.detectByHue(bench.right, telemetry) == colorsensors.DetectedColor.PURPLE
                && bench.detectByHue(bench.left, telemetry) == colorsensors.DetectedColor.PURPLE) {
            performShotSequence(false, false);
            sleep(500);
            rotateGreenIn();
            sleep(500);
            shootgreen();
        } else {
            rotatePurpleIn();
            if (!shootpurple()) {
                rotatePurpleIn();
                sleep(500);
                shootpurple();
            }
            sleep(500);
            if (!shootpurple()) {
                rotatePurpleIn();
                sleep(500);
                shootpurple();
            }
            sleep(500);
            if (!shootgreen()) {
                rotateGreenIn();
                sleep(500);
                shootgreen();
            }
        }
    }

    private void performShotSequence(boolean leftFirst, boolean isGPP) {
        resources.lgun.setPower(1);
        resources.rgun.setPower(-1);
        sleep(800);
        if (isGPP) {
            resources.llift.setPosition(0.625);
            sleep(600);
            resources.rlift.setPosition(0.3);
        } else {
            resources.rlift.setPosition(0.3);
            sleep(500);
            resources.llift.setPosition(0.625);
        }
        sleep(600);
        resources.lgun.setPower(0);
        resources.llift.setPosition(1);
        resources.rlift.setPosition(0.005);
        resources.rgun.setPower(0);
    }

    private void shootleft() {
        resources.lgun.setPower(1);
        sleep(1000);
        resources.llift.setPosition(0.625);
        sleep(1200);
        resources.lgun.setPower(0);
        resources.llift.setPosition(1);
        sleep(500);
    }

    private void shootright() {
        resources.rgun.setPower(-1);
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
}