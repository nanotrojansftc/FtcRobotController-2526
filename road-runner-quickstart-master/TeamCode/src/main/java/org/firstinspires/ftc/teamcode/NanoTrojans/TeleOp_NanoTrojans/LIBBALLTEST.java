package org.firstinspires.ftc.teamcode.NanoTrojans.TeleOp_NanoTrojans;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.DropBalls;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top;

@TeleOp(name = "LIBBALLTEST")
public class LIBBALLTEST extends LinearOpMode {

    @Override
    public void runOpMode() {



        // --- Minimal init needed for DropBalls.rotate() ---
        resources_top resources = new resources_top(hardwareMap); // not used by rotate(), but constructor requires it
        colorsensors bench = new colorsensors();                  // not used by rotate(), but constructor requires it
        bench.init(hardwareMap);                                  // safe to init anyway

        CRServo fspin = hardwareMap.crservo.get("fspin");
        CRServo rspin = hardwareMap.crservo.get("rspin");
        CRServo lspin = hardwareMap.crservo.get("lspin");

        DropBalls dropBalls = new DropBalls(this, resources, bench, fspin, rspin, lspin);

        telemetry.addLine("Ready. Press PLAY to rotate once.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetry.addLine("Rotating...");
        telemetry.update();

        dropBalls.rotateGreenIn();   // ✅ calls your helper function once

        telemetry.addLine("Done. (Stop the OpMode)");
        telemetry.update();

        // Keep OpMode alive so you can see telemetry
        while (opModeIsActive()) {
            sleep(50);
        }
    }
}
