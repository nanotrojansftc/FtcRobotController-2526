package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class resources_top_MainRobot {
    public DcMotor shooter1 = null;
    public DcMotor shooter2 = null;
    public DcMotor intake = null;


//    public CRServo rhs = null;
//    public CRServo lhs = null;
//    //servo motors
//    public Servo blocker = null;
//    public Servo ril = null;
//    public Servo lil = null;
//
//    public Servo claw = null;
//    public Servo ra = null;
//    public Servo la = null;
////    public DcMotor hanger = null;
//    public Servo backclaw = null;


    public resources_top_MainRobot(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.dcMotor.get("shooter1");
        shooter2 = hardwareMap.dcMotor.get("shooter2");
        intake = hardwareMap.dcMotor.get("intake");

//        rhs = hardwareMap.crservo.get("righthorizontal");
//        lhs = hardwareMap.crservo.get("lefthorizontal");
//
//        blocker = hardwareMap.servo.get("blocker");
//
//        lil = hardwareMap.servo.get("lintake");
//        ril = hardwareMap.servo.get("rintake");
//
//        claw = hardwareMap.servo.get("claw");
//
//        la = hardwareMap.servo.get("leftarmservo");
//        ra = hardwareMap.servo.get("rightarmservo");
//
////        hanger = hardwareMap.dcMotor.get("hang");
//
//        backclaw = hardwareMap.servo.get("backclaw");
    }
}
