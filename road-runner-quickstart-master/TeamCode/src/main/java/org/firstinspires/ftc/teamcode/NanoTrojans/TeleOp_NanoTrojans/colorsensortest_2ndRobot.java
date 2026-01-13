package org.firstinspires.ftc.teamcode.NanoTrojans.TeleOp_NanoTrojans;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors_2ndRobot;

@TeleOp(name ="colorsensortest_2ndRobot", group = "TeleOp")
public class colorsensortest_2ndRobot extends OpMode {
    colorsensors_2ndRobot bench = new colorsensors_2ndRobot();
    colorsensors_2ndRobot.DetectedColor first;
    colorsensors_2ndRobot.DetectedColor second;
      colorsensors_2ndRobot.DetectedColor third;

    colorsensors_2ndRobot.DetectedColor detectcolor;
//    colorsensors.DetectedColor detectedColor;
//    colorsensors.

    @Override
    public void init(){
        bench.init(hardwareMap);

    }
    @Override
    public void loop(){
//        telemetry.addData("left ", bench.getDetectedColor(telemetry));
//        telemetry.addData("right", bench.getrightcolor(telemetry));
//        telemetry.addData("front", bench.getbackcolor(telemetry));

//       bench.getDetectedColor(telemetry);
//       bench.getrightcolor(telemetry);
//       bench.getbackcolor(telemetry);

        first = bench.getFirstColor(telemetry);
        second = bench.getSecondColor(telemetry);

        third = bench.getThirdColor(telemetry);

        telemetry.addData("First color", first);
        telemetry.addData("Second color", second);
        telemetry.addData("Third color", third);

        telemetry.addData("Detect by hue", "----------");
        float hue1 = bench.getHueValue(bench.colorsensor1,telemetry);
        float hue2 = bench.getHueValue(bench.colorsensor2,telemetry);
        float hue3 = bench.getHueValue(bench.colorsensor3,telemetry);

        telemetry.addData("First color", "%s, Hue=%.2f" , bench.detectByHue(hue1, telemetry),hue1);
        telemetry.addData("Second color", "%s,Hue=%.2f" ,bench.detectByHue(hue2, telemetry),hue2);
        telemetry.addData("Third color", "%s,Hue=%.2f" , bench.detectByHue(hue3, telemetry),hue3);

        telemetry.addData("Detect by hue 2", "----------");
        telemetry.addData("First Color", bench.detectByHue(bench.colorsensor1, telemetry));
        telemetry.addData("Second Color", bench.detectByHue(bench.colorsensor2, telemetry));
        telemetry.addData("Third Color", bench.detectByHue(bench.colorsensor3, telemetry));

        telemetry.update();
    }
}
