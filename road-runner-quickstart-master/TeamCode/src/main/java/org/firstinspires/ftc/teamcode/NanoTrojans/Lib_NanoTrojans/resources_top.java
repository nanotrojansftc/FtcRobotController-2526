package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class resources_top {
    public DcMotor lgun;
    public DcMotor rgun;
    public DcMotor intake;
    public Servo llift;
    public Servo rlift;
    public CRServo fspin;
    public CRServo rspin;
    public CRServo lspin;

    public int apriltagvalue = 0;
    public int towervalue = 0;

    public resources_top(HardwareMap hardwareMap) {
        lgun = hardwareMap.get(DcMotor.class, "lgun");
        rgun = hardwareMap.get(DcMotor.class, "rgun");
        intake = hardwareMap.get(DcMotor.class, "intake");
        llift = hardwareMap.get(Servo.class, "llift");
        rlift = hardwareMap.get(Servo.class, "rlift");
        fspin = hardwareMap.get(CRServo.class, "fspin");
        rspin = hardwareMap.get(CRServo.class, "rspin");
        lspin = hardwareMap.get(CRServo.class, "lspin");

        lgun.setDirection(DcMotorSimple.Direction.REVERSE);
        rgun.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        llift.setPosition(1);
        rlift.setPosition(0.005);
    }
}