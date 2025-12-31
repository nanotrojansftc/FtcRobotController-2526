package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class controls_top202425 {
    // Constants for encoder counts and wheel measurements
    static final double COUNTS_PER_REVOLUTION = 537.7; // Encoder counts per revolution
    static final double WHEEL_DIAMETER_MM = 96.0; // Wheel diameter in millimeters
    static final double MM_PER_REVOLUTION = WHEEL_DIAMETER_MM * Math.PI; // Wheel circumference
    static final double COUNTS_PER_MM = COUNTS_PER_REVOLUTION / MM_PER_REVOLUTION; // Counts per millimeter
    static final double COUNTS_PER_INCH = COUNTS_PER_MM * 25.4; // Counts per inch

    //private DcMotor intake = null;
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
    //private CRServo robotLift = null;

    public Servo backclaw = null;
    //private CRServo robotLift = null;



    public controls_top202425(DcMotor lsR, DcMotor lsL, CRServo lhorizontal, CRServo rhorizontal, DcMotor intak
            , Servo blocke, Servo ri, Servo li, Servo cla, Servo r, Servo l, Servo bclaw)
    {
        claw=cla;
        intake= intak;
        blocker = blocke;
        ril = ri;
        lil = li;
        lsRight=lsR;
        ra=r;
        la=l;
        lsLeft=lsL;
        lhs = lhorizontal;
        rhs = rhorizontal;
//        hanger = hang;
        backclaw =bclaw;
    }
    public void closeclaw()
    {
        //for the claw, it is a regular motor so you set positions; you just have to keep tweaking the code and test out positions that you input.
        claw.setPosition(0.7);
    }
    public void openclaw()
    {
        //for the claw, it is a regular motor so you set positions; you just have to keep tweaking the code and test out positions that you input.
        claw.setPosition(0.3);
    }
    public void armup()
    {
        ra.setPosition(0.01);
        la.setPosition(0.2);
    }
    public void armdown()
    {
        ra.setPosition(0.9);
        la.setPosition(1);
    }
    public void intakedown()
    {
        // also added blocker up
        blocker.setPosition(1);

        ril.setPosition(0.95);
        lil.setPosition(0.0);
    }
    public void intakeup()
    {
        // also added blocker down
        blocker.setPosition(0.58);

        ril.setPosition(0.4);
        lil.setPosition(0.6);
    }
    public void intakeon()
    {
        intake.setPower(1);
    }
    public void intakehalf()
    {
        intake.setPower(0.65);
    }
    public void intakeoff()
    {
        intake.setPower(0);
    }
    public void intakereverse()
    {
        intake.setPower(-1);
    }
    public void hsretract()
    {
        rhs.setPower(0.1);
        lhs.setPower(-0.1);
    }

    public void hsstall()
    {
        rhs.setPower(0.05);
        lhs.setPower(-0.05);
    }

    public void hsextend()
    {
        rhs.setPower(-0.3);
        lhs.setPower(0.3);
    }
    public void lson()
    {
        lsRight.setPower(-1);
        lsLeft.setPower(1);
    }

    public void lson80wpower()
    {
        lsRight.setPower(-0.8);
        lsLeft.setPower(0.8);
    }
    public void lsonlowpower()
    {
        lsRight.setPower(-0.5);
        lsLeft.setPower(0.5);
    }
    public void lsreverse()
    {
        lsRight.setPower(1);
        lsLeft.setPower(-1);
    }

    public void lsreverseMiddle()
    {
        lsRight.setPower(0.9);
        lsLeft.setPower(-0.9);
    }
    public void lsstall()
    {
        lsRight.setPower(-0.1);
        lsLeft.setPower(0.1);
    }
    public void lsoff(){
        lsRight.setPower(0);
        lsLeft.setPower(0);
    }
//    public void hangeron()
//    {
//        hanger.setPower(1);
//    }
//    public void hangeroff()
//    {
//        hanger.setPower(0);
//    }
//    public void hangerreverse()
//    {
//        hanger.setPower(-1);
//    }

    public void bclawopen()
    {
        backclaw.setPosition(1);
    }
    public void bclawclose()
    {
        backclaw.setPosition(0);

    }

}