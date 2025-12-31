package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.Control_Base_NT;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.controls_top;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_base;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_top;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class colorsensors {
    NormalizedColorSensor left, right, back;


    public enum DetectedColor{
        GREEN,
        PURPLE,
        UNKNOWN
    }
    public enum rightcolor{
        GREEN,
        PURPLE,
        UNKNOWN
    }
    public enum backcolor{
        GREEN,
        PURPLE,
        UNKNOWN
    }


    public void init(HardwareMap hardwareMap){
        left = hardwareMap.get(NormalizedColorSensor.class, "lgunsensor");
        right = hardwareMap.get(NormalizedColorSensor.class, "rgunsensor");
        back = hardwareMap.get(NormalizedColorSensor.class, "intake sensor");
        left.setGain(80);
        right.setGain(50);
        back.setGain(100);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA l = left.getNormalizedColors();// return 4 values, red, green, blue, alpha

        float normRed, normGreen, normBlue;
        normRed = l.red / l.alpha;
        normGreen = l.green / l.alpha;
        normBlue = l.blue / l.alpha;
//        telemetry.addLine("Left");
//        telemetry.addData("red", normRed);
//        telemetry.addData("green",normGreen);
//        telemetry.addData("blue", normBlue);

        if (normRed < 0.53 && normGreen > 1 && normBlue > 0.83) {
            return DetectedColor.GREEN;
        }
        else if (normRed>0.53 && normGreen >0.9 && normBlue <1.15){
            return DetectedColor.PURPLE;
        }
        else{
            return DetectedColor.UNKNOWN;
        }


    }
    public rightcolor getrightcolor(Telemetry telemetry) {

        NormalizedRGBA r = right.getNormalizedColors();
        float rr, rg, rb;
        rr = r.red / r.alpha;
        rg = r.green / r.alpha;
        rb = r.blue / r.alpha;
//        telemetry.addLine("Right");
//        telemetry.addData("red", rr);
//        telemetry.addData("green", rg);
//        telemetry.addData("blue", rb);
        if (rr < 0.455 && rg > 0.93 && rb > 0.9){
            return rightcolor.GREEN;
        }
        else if (rr>0.45 && rg <0.93 && rb >0.9){
            return rightcolor.PURPLE;
        }
        else {
            return rightcolor.UNKNOWN;
        }
    }
    public backcolor getbackcolor(Telemetry telemetry){
        NormalizedRGBA b = back.getNormalizedColors();
        float br, bg, bb;
        br = b.red / b.alpha;
        bg = b.green / b.alpha;
        bb = b.blue / b.alpha;
//        telemetry.addLine("front");
//        telemetry.addData("red", br);
//        telemetry.addData("green", bg);
//        telemetry.addData("blue", bb);


        if (br <0.8 && bg > 1.47 && bb > 1.2){
            return backcolor.GREEN;
        }
        else if (br>0.8 && bg <1.4 && bb >1.3){
            return backcolor.PURPLE;
        }
        else {
            return backcolor.UNKNOWN;
        }

    }





}
