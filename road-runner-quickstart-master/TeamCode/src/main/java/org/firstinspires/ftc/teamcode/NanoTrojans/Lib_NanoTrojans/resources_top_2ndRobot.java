package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class resources_top_2ndRobot {
    public DcMotor shooter = null;
    public DcMotor intake = null;



//    public CRServo rhs = null;
//    public CRServo lhs = null;
//    //servo motors
    public Servo lift1 = null;
    public Servo lift2 = null;
    public Servo lift3 = null;


    public resources_top_2ndRobot(HardwareMap hardwareMap) {
        shooter = hardwareMap.dcMotor.get("shooter");
        intake = hardwareMap.dcMotor.get("intake");

         lift1 = hardwareMap.servo.get("lift1");
         lift2 = hardwareMap.servo.get("lift2");
        lift3 = hardwareMap.servo.get("lift3");


    }
}
