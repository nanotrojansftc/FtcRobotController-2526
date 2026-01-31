package org.firstinspires.ftc.teamcode.NanoTrojans.Tools;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;

@TeleOp(name ="colorsensortest", group = "TeleOp")
public class colorsensortest extends OpMode {

    colorsensors bench = new colorsensors();

    @Override
    public void init(){
        bench.init(hardwareMap);
    }

    @Override
    public void loop(){
        telemetry.addData("Status", "Running Test...");
        telemetry.addLine("-----------------------");

        // 1. Process Left Sensor
        displaySensorInfo("LEFT", bench.left);

        // 2. Process Right Sensor
        displaySensorInfo("RIGHT", bench.right);

        // 3. Process Back Sensor
        displaySensorInfo("BACK", bench.back);

        telemetry.update();
    }

    // Helper method to keep the loop clean
    public void displaySensorInfo(String name, com.qualcomm.robotcore.hardware.NormalizedColorSensor sensor) {
        // Get the data from the library
        colorsensors.DetectedColor color = bench.detectByHue(sensor, telemetry);
        float hue = bench.getHue(sensor);
        int[] rgb = bench.getRGB(sensor);

        // Format and Print
        telemetry.addData(name, "%s | Hue: %.1f | RGB: %d, %d, %d",
                color, hue, rgb[0], rgb[1], rgb[2]);
    }
}