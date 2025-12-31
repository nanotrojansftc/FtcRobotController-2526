package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class resources_top202425 {
    public DcMotor lsRight = null;
    public DcMotor lsLeft = null;
    public CRServo rhs = null;
    public CRServo lhs = null;
    public DcMotor intake = null;

    //servo motors
    public Servo blocker = null;
    public Servo ril = null;
    public Servo lil = null;

    public Servo claw = null;
    public Servo ra = null;
    public Servo la = null;
//    public DcMotor hanger = null;
    public Servo backclaw = null;


    public resources_top202425(HardwareMap hardwareMap) {
        lsRight = hardwareMap.dcMotor.get("lsRight");
        lsLeft = hardwareMap.dcMotor.get("lsLeft");

        intake = hardwareMap.dcMotor.get("intake");

        rhs = hardwareMap.crservo.get("righthorizontal");
        lhs = hardwareMap.crservo.get("lefthorizontal");

        blocker = hardwareMap.servo.get("blocker");

        lil = hardwareMap.servo.get("lintake");
        ril = hardwareMap.servo.get("rintake");

        claw = hardwareMap.servo.get("claw");

        la = hardwareMap.servo.get("leftarmservo");
        ra = hardwareMap.servo.get("rightarmservo");

//        hanger = hardwareMap.dcMotor.get("hang");

        backclaw = hardwareMap.servo.get("backclaw");
    }
}
