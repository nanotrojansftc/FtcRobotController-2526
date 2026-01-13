package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class colorsensors_2ndRobot {
   public NormalizedColorSensor colorsensor1,colorsensor2,colorsensor3;


    public enum DetectedColor{
        GREEN,
        PURPLE,
        UNKNOWN
    }


    public void init(HardwareMap hardwareMap){
        colorsensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor1");
        colorsensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor2");
        colorsensor3 = hardwareMap.get(NormalizedColorSensor.class, "sensor3");
        colorsensor1.setGain(80);
        colorsensor2.setGain(50);
        colorsensor3.setGain(100);
    }

    public DetectedColor getFirstColor(Telemetry telemetry) {
        NormalizedRGBA l = colorsensor1.getNormalizedColors();// return 4 values, red, green, blue, alpha

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

      public DetectedColor getSecondColor(Telemetry telemetry) {

        NormalizedRGBA r = colorsensor2.getNormalizedColors();
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


    public DetectedColor getThirdColor(Telemetry telemetry){
        NormalizedRGBA b = colorsensor3.getNormalizedColors();
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

            if (hue >= 154 && hue <= 175) {
                result = DetectedColor.GREEN;
                //return DetectedColor.GREEN;
            } else if (hue > 175 && hue <= 240) {

                result = DetectedColor.PURPLE;
                //return DetectedColor.PURPLE;
            }

        return result;
        //return DetectedColor.UNKNOWN;
    }

    public DetectedColor detectByHue(float hue, Telemetry telemetry) {


        DetectedColor result = DetectedColor.UNKNOWN;
        if (hue >= 154 && hue <= 175) {
            result = DetectedColor.GREEN;
            //return DetectedColor.GREEN;
        } else if (hue > 175 && hue <= 240) {

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
