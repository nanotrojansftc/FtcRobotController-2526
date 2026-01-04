package org.firstinspires.ftc.teamcode.NanoTrojans.TeleOp_NanoTrojans;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;

@TeleOp(name ="colorsensortest", group = "TeleOp")
public class colorsensortest extends OpMode {
    colorsensors bench = new colorsensors();
    colorsensors.DetectedColor left;
    colorsensors.DetectedColor right;
      colorsensors.DetectedColor back;

    colorsensors.DetectedColor detectcolor;
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

        left = bench.getDetectedColor(telemetry);
        right = bench.getrightcolor(telemetry);

        back = bench.getbackcolor(telemetry);

        telemetry.addData("Left color", left);
        telemetry.addData("Right color", right);
        telemetry.addData("Back color", back);

        telemetry.addData("Detect by hue", "----------");
        float lefthue = bench.getHueValue(bench.left,telemetry);
        float righthue = bench.getHueValue(bench.right,telemetry);
        float backhue = bench.getHueValue(bench.back,telemetry);

        telemetry.addData("Left color", "%s, Hue=%.2f" , bench.detectByHue(lefthue, telemetry),lefthue);
        telemetry.addData("Right color", "%s,Hue=%.2f" ,bench.detectByHue(righthue, telemetry),righthue);
        telemetry.addData("Back color", "%s,Hue=%.2f" , bench.detectByHue(backhue, telemetry),backhue);

        telemetry.addData("Detect by hue 2", "----------");
        telemetry.addData("Left Color", bench.detectByHue(bench.left, telemetry));
        telemetry.addData("right Color", bench.detectByHue(bench.right, telemetry));
        telemetry.addData("back Color", bench.detectByHue(bench.back, telemetry));

        telemetry.update();
    }
}
