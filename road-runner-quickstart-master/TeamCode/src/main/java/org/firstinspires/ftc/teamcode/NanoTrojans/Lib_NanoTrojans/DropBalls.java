package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

// --- CORRECTED IMPORTS ---


public class DropBalls {
    private resources_top resources;
    private colorsensors bench;
    private final LinearOpMode opMode;  // ✅ add this
    private int carousel;

    private CRServo fspin;
    private CRServo rspin;
    private CRServo lspin;
    public DropBalls(
            LinearOpMode opMode,
            resources_top resources,
            colorsensors bench,
            CRServo fspin,
            CRServo rspin,
            CRServo lspin
    ) {
        this.opMode = opMode;
        this.resources = resources;
        this.bench = bench;
        this.fspin = fspin;
        this.rspin = rspin;
        this.lspin = lspin;
    }

    // OPTIONAL: one single entry method
    public void shootByTag(int tagId) {
        if (tagId == 21) shootGPP();
        else if (tagId == 22) shootPGP();
        else if (tagId == 23) shootPPG();
    }

    public void shootGPP() {
        if ((bench.detectByHue(bench.left, opMode.telemetry) == colorsensors.DetectedColor.GREEN
                && bench.detectByHue(bench.right, opMode.telemetry) == colorsensors.DetectedColor.PURPLE)) {
            performShotSequence(true, true);
            opMode.sleep(500);
            rotatePurpleIn();
            opMode.sleep(500);
            shootpurple();
        } else if (bench.detectByHue(bench.right, opMode.telemetry) == colorsensors.DetectedColor.GREEN
                && bench.detectByHue(bench.left, opMode.telemetry) == colorsensors.DetectedColor.PURPLE) {
            performShotSequence(false, true);
            opMode.sleep(500);
            rotatePurpleIn();
            opMode.sleep(500);
            shootpurple();
        } else {
            rotateGreenIn();
            if (!shootgreen()) {
                rotateGreenIn();
                opMode.sleep(500);
                shootgreen();
            }
            opMode.sleep(500);
            if (!shootpurple()) {
                rotatePurpleIn();
                opMode.sleep(500);
                shootpurple();
            }
            opMode.sleep(500);
            if (!shootpurple()) {
                rotatePurpleIn();
                opMode.sleep(500);
                shootpurple();
            }
        }
    }

    public void shootPGP() {
        if ((bench.detectByHue(bench.left, opMode.telemetry) == colorsensors.DetectedColor.GREEN
                && bench.detectByHue(bench.right, opMode.telemetry) == colorsensors.DetectedColor.PURPLE)) {
            performShotSequence(true, false);
            opMode.sleep(500);
            rotatePurpleIn();
            opMode.sleep(500);
            shootpurple();
        } else if (bench.detectByHue(bench.right, opMode.telemetry) == colorsensors.DetectedColor.GREEN
                && bench.detectByHue(bench.left, opMode.telemetry) == colorsensors.DetectedColor.PURPLE) {
            performShotSequence(false, false);
            opMode.sleep(500);
            rotatePurpleIn();
            opMode.sleep(500);
            shootpurple();
        } else {
            rotatePurpleIn();
            if (!shootpurple()) {
                rotatePurpleIn();
                opMode.sleep(500);
                shootpurple();
            }
            opMode.sleep(500);
            if (!shootgreen()) {
                rotateGreenIn();
                opMode.sleep(500);
                shootgreen();
            }
            opMode.sleep(500);
            if (!shootpurple()) {
                rotatePurpleIn();
                opMode.sleep(500);
                shootpurple();
            }
        }
    }

    public void shootPPG() {
        if ((bench.detectByHue(bench.left, opMode.telemetry) == colorsensors.DetectedColor.PURPLE
                && bench.detectByHue(bench.right, opMode.telemetry) == colorsensors.DetectedColor.PURPLE)) {
            performShotSequence(true, false);
            opMode.sleep(500);
            rotateGreenIn();
            opMode.sleep(500);
            shootgreen();
        } else if (bench.detectByHue(bench.right, opMode.telemetry) == colorsensors.DetectedColor.PURPLE
                && bench.detectByHue(bench.left, opMode.telemetry) == colorsensors.DetectedColor.PURPLE) {
            performShotSequence(false, false);
            opMode.sleep(500);
            rotateGreenIn();
            opMode.sleep(500);
            shootgreen();
        } else {
            rotatePurpleIn();
            if (!shootpurple()) {
                rotatePurpleIn();
                opMode.sleep(500);
                shootpurple();
            }
            opMode.sleep(500);
            if (!shootpurple()) {
                rotatePurpleIn();
                opMode.sleep(500);
                shootpurple();
            }
            opMode.sleep(500);
            if (!shootgreen()) {
                rotateGreenIn();
                opMode.sleep(500);
                shootgreen();
            }
        }
    }

    public void performShotSequence(boolean leftFirst, boolean isGPP) {
        resources.lgun.setPower(1);
        resources.rgun.setPower(-1);
        opMode.sleep(800);
        if (isGPP) {
            resources.llift.setPosition(0.625);
            opMode.sleep(600);
            resources.rlift.setPosition(0.3);
        } else {
            resources.rlift.setPosition(0.3);
            opMode.sleep(500);
            resources.llift.setPosition(0.625);
        }
        opMode.sleep(600);
        resources.lgun.setPower(0);
        resources.llift.setPosition(1);
        resources.rlift.setPosition(0.005);
        resources.rgun.setPower(0);
    }

    public void shootleft() {
        resources.lgun.setPower(1);
        opMode.sleep(1000);
        resources.llift.setPosition(0.625);
        opMode.sleep(1200);
        resources.lgun.setPower(0);
        resources.llift.setPosition(1);
        opMode.sleep(500);
    }
    public void shootright() { /* paste + replace sleep */
        resources.rgun.setPower(-1);
        opMode.sleep(1000);
        resources.rlift.setPosition(0.3);
        opMode.sleep(800);
        resources.rlift.setPosition(0.005);
        resources.rgun.setPower(0);
        opMode.sleep(500);
    }

    public boolean shootgreen() { /* paste + replace sleep/opMode.opMode.telemetry */
        if (bench.detectByHue(bench.left, opMode.telemetry) == colorsensors.DetectedColor.GREEN) {
            shootleft();
        } else if (bench.detectByHue(bench.right, opMode.telemetry) == colorsensors.DetectedColor.GREEN) {
            shootright();
        } else {
            return false;
        }
        return true;
    }
    public boolean shootpurple() {
        if (bench.detectByHue(bench.left, opMode.telemetry) == colorsensors.DetectedColor.PURPLE) {
            shootleft();
        } else if (bench.detectByHue(bench.right, opMode.telemetry) == colorsensors.DetectedColor.PURPLE) {
            shootright();
        } else {
            return false;
        }
        return true;
    }

    public void rotate() {
        if (carousel == 2) {
            carousel = 0;
            lspin.setPower(-1);
            opMode.sleep(474);
            lspin.setPower(0);
        } else if (carousel == 1) {
            carousel += 1;
            rspin.setPower(-1);
            opMode.sleep(474);
            rspin.setPower(0);
        } else if (carousel == 0) {
            carousel += 1;
            fspin.setPower(-1);
            opMode.sleep(474);
            fspin.setPower(0);
        }
    }

    public boolean rotatePurpleIn() {
        rotate();
        opMode.sleep(500);
        if (!(bench.detectByHue(bench.right, opMode.telemetry) == colorsensors.DetectedColor.PURPLE
                || bench.detectByHue(bench.left, opMode.telemetry) == colorsensors.DetectedColor.PURPLE)) {
            rotate();
            opMode.sleep(500);
        }
        if (!(bench.detectByHue(bench.right, opMode.telemetry) == colorsensors.DetectedColor.PURPLE
                || bench.detectByHue(bench.left, opMode.telemetry) == colorsensors.DetectedColor.PURPLE)) {
            rotate();
            opMode.sleep(500);
        }
        return bench.detectByHue(bench.right, opMode.telemetry) == colorsensors.DetectedColor.PURPLE
                || bench.detectByHue(bench.left, opMode.telemetry) == colorsensors.DetectedColor.PURPLE;
    }
    public boolean rotateGreenIn() {
        rotate();
        opMode.sleep(500);
        if (!(bench.detectByHue(bench.right, opMode.telemetry) == colorsensors.DetectedColor.GREEN
                || bench.detectByHue(bench.left, opMode.telemetry) == colorsensors.DetectedColor.GREEN)) {
            rotate();
            opMode.sleep(500);
        }
        if (!(bench.detectByHue(bench.right, opMode.telemetry) == colorsensors.DetectedColor.GREEN
                || bench.detectByHue(bench.left, opMode.telemetry) == colorsensors.DetectedColor.GREEN)) {
            rotate();
            opMode.sleep(500);
        }
        return bench.detectByHue(bench.right, opMode.telemetry) == colorsensors.DetectedColor.GREEN
                || bench.detectByHue(bench.left, opMode.telemetry) == colorsensors.DetectedColor.GREEN;
    }
}
