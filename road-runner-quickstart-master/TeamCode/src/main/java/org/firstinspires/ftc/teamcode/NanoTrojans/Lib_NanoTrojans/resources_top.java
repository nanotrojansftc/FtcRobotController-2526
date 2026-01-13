package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class resources_top {
    public DcMotor rgun = null;
    public DcMotor lgun = null;
    public DcMotor intake = null;


//    public CRServo rhs = null;
//    public CRServo lhs = null;
//    //servo motors
    public Servo llift = null;
    public Servo rlift = null;
    public CRServo fspin = null;
    public CRServo rspin = null;
    public CRServo lspin = null;

    public int apriltagvalue =21;

    public int towervalue = 20;



    public resources_top(HardwareMap hardwareMap) {
        lgun = hardwareMap.dcMotor.get("lgun");
        rgun = hardwareMap.dcMotor.get("rgun");
        intake = hardwareMap.dcMotor.get("intake");

         llift = hardwareMap.servo.get("llift");
         rlift = hardwareMap.servo.get("rlift");
         fspin = hardwareMap.crservo.get("fspin");
         rspin= hardwareMap.crservo.get("rspin");
         lspin = hardwareMap.crservo.get("lspin");

    }
}
