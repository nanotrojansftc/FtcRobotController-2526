package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;

@TeleOp(name ="testtransfer", group = "TeleOp")
public class testtransfer extends LinearOpMode {
    colorsensors bench = new colorsensors();
    colorsensors.DetectedColor left;
    colorsensors.rightcolor right;
    colorsensors.backcolor back;
    //    colorsensors.DetectedColor detectedColor;
//    colorsensors.



    public void runOpMode() throws InterruptedException {
        bench.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            left = bench.getDetectedColor(telemetry);
            right = bench.getrightcolor(telemetry);
            back = bench.getbackcolor(telemetry);

            telemetry.addData("Left color", left);
            telemetry.addData("Right color", right);
            telemetry.addData("Back color", back);
        }





    }}
