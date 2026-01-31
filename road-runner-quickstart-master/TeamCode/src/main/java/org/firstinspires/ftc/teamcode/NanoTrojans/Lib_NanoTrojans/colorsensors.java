package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import android.graphics.Color;

public class colorsensors {
    public NormalizedColorSensor left, right, back;

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public void init(HardwareMap hardwareMap) {
        // Make sure these names match your Robot Configuration exactly
        left = hardwareMap.get(NormalizedColorSensor.class, "lgunsensor");
        right = hardwareMap.get(NormalizedColorSensor.class, "rgunsensor");
        back = hardwareMap.get(NormalizedColorSensor.class, "intake sensor");

        left.setGain(80);
        right.setGain(50);
        back.setGain(100);
    }

    // --- MAIN DETECTION METHOD ---
    public DetectedColor detectByHue(NormalizedColorSensor sensor, Telemetry telemetry) {
        float hue = getHue(sensor);

        if (sensor == left) {
            return detectLeft(hue);
        } else if (sensor == right) {
            return detectRight(hue);
        } else if (sensor == back) {
            return detectBack(hue);
        }
        return DetectedColor.UNKNOWN;
    }

    // --- HELPER METHODS FOR DATA ---
    public float getHue(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.RGBToHSV((int)(c.red * 255), (int)(c.green * 255), (int)(c.blue * 255), hsv);
        return hsv[0];
    }

    public int[] getRGB(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        return new int[]{
                (int)(c.red * 255),
                (int)(c.green * 255),
                (int)(c.blue * 255)
        };
    }

    // --- INTERNAL LOGIC ---
    private DetectedColor detectLeft(float hue) {
        if (hue >= 140 && hue <= 170) return DetectedColor.GREEN;
        if (hue >= 171 && hue <= 240) return DetectedColor.PURPLE;
        return DetectedColor.UNKNOWN;
    }

    private DetectedColor detectRight(float hue) {
        if (hue >= 140 && hue <= 170) return DetectedColor.GREEN;
        if (hue >= 171 && hue <= 240) return DetectedColor.PURPLE;
        return DetectedColor.UNKNOWN;
    }

    private DetectedColor detectBack(float hue) {
        if (hue >= 140 && hue <= 170) return DetectedColor.GREEN;
        if (hue >= 171 && hue <= 240) return DetectedColor.PURPLE;
        return DetectedColor.UNKNOWN;
    }

    // Legacy support methods
    public DetectedColor getDetectedColor(Telemetry telemetry) { return detectByHue(left, telemetry); }
    public DetectedColor getrightcolor(Telemetry telemetry) { return detectByHue(right, telemetry); }
    public DetectedColor getbackcolor(Telemetry telemetry) { return detectByHue(back, telemetry); }

    public DetectedColor detectByHue(float hue, Telemetry telemetry) {
        if (hue >= 140 && hue <= 170) return DetectedColor.GREEN;
        if (hue >= 171 && hue <= 240) return DetectedColor.PURPLE;
        return DetectedColor.UNKNOWN;
    }
}