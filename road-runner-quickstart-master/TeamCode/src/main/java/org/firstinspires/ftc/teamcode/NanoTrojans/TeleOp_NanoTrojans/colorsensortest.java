package org.firstinspires.ftc.teamcode.NanoTrojans.TeleOp_NanoTrojans;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.colorsensors;

@TeleOp(name ="colorsensortest", group = "TeleOp")
public class colorsensortest extends OpMode {
    colorsensors bench = new colorsensors();
    colorsensors.DetectedColor left;
    colorsensors.rightcolor right;
    colorsensors.backcolor back;
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

    }
}
