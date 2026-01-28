package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;
import com.qualcomm.hardware.lynx.commands.core.LynxFtdiResetQueryResponse;
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
import android.graphics.Color;

public class colorsensors {
   public NormalizedColorSensor left, right, back;


    public enum DetectedColor{
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

      public DetectedColor getrightcolor(Telemetry telemetry) {

        NormalizedRGBA r = right.getNormalizedColors();
        float rr, rg, rb;
        rr = r.red / r.alpha;
        rg = r.green / r.alpha;
        rb = r.blue / r.alpha;

        if (rr < 0.455 && rg > 0.93 && rb > 0.9){
            return DetectedColor.GREEN;
        }
        else if (rr>0.45 && rg <0.93 && rb >0.9){
            return DetectedColor.PURPLE;
        }
        else {
            return DetectedColor.UNKNOWN;
        }
    }


    public DetectedColor getbackcolor(Telemetry telemetry){
        NormalizedRGBA b = back.getNormalizedColors();
        float br, bg, bb;
        br = b.red / b.alpha;
        bg = b.green / b.alpha;
        bb = b.blue / b.alpha;

        if (br <0.8 && bg > 1.47 && bb > 1.2){
            return DetectedColor.GREEN;
        }
        else if (br>0.8 && bg <1.4 && bb >1.3){
            return DetectedColor.PURPLE;
        }
        else {
            return DetectedColor.UNKNOWN;
        }

    }

    public DetectedColor detectByHue(NormalizedColorSensor sensor, Telemetry telemetry) {

        NormalizedRGBA c = sensor.getNormalizedColors();
        DetectedColor result = DetectedColor.UNKNOWN;
        // Convert normalized RGB (0–1) → 0–255
        int r = (int)(c.red   * 255);
        int g = (int)(c.green * 255);
        int b = (int)(c.blue  * 255);

        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        float hue = hsv[0];        // 0–360
        float sat = hsv[1];        // 0–1
        float val = hsv[2];        // 0–1

        // Filter out white / dark
//        if (sat < 0.3 || val < 0.2) {
//            result = DetectedColor.UNKNOWN;
//        }
//        else
//        {
            if (hue >= 140 && hue <= 170) {// Changed from 149 to 163 and then 170
                result = DetectedColor.GREEN;
                //return DetectedColor.GREEN;
            } else if (hue >= 171 && hue <= 240) { // changed from 163

                result = DetectedColor.PURPLE;
                //return DetectedColor.PURPLE;
            }
//        }
//        telemetry.addData("color", "%s, Hue=%.2f, S=%.2f, v=%.2f" , result,hue, sat, val);
//        telemetry.update();
        return result;
        //return DetectedColor.UNKNOWN;
    }

    public DetectedColor detectByHue(float hue, Telemetry telemetry) {


        DetectedColor result = DetectedColor.UNKNOWN;
        if (hue >= 140 && hue <= 170) { // changed from 163
            result = DetectedColor.GREEN;
            //return DetectedColor.GREEN;
        } else if (hue >= 171 && hue <= 240) { // changed from 163

            result = DetectedColor.PURPLE;
            //return DetectedColor.PURPLE;
        }

        return result;
    }

    public float getHueValue(NormalizedColorSensor sensor, Telemetry telemetry) {

        NormalizedRGBA c = sensor.getNormalizedColors();
        DetectedColor result = DetectedColor.UNKNOWN;
        // Convert normalized RGB (0–1) → 0–255
        int r = (int)(c.red   * 255);
        int g = (int)(c.green * 255);
        int b = (int)(c.blue  * 255);

        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        float hue = hsv[0];        // 0–360
        float sat = hsv[1];        // 0–1
        float val = hsv[2];        // 0–1


        return hue;
        //return DetectedColor.UNKNOWN;
    }



}
