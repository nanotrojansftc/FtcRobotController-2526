package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;


import com.qualcomm.robotcore.hardware.DcMotor;

public class controls_top_MainRobot {
    // Constants for encoder counts and wheel measurements
    static final double COUNTS_PER_REVOLUTION = 537.7; // Encoder counts per revolution
    static final double WHEEL_DIAMETER_MM = 96.0; // Wheel diameter in millimeters
    static final double MM_PER_REVOLUTION = WHEEL_DIAMETER_MM * Math.PI; // Wheel circumference
    static final double COUNTS_PER_MM = COUNTS_PER_REVOLUTION / MM_PER_REVOLUTION; // Counts per millimeter
    static final double COUNTS_PER_INCH = COUNTS_PER_MM * 25.4; // Counts per inch

    //private DcMotor intake = null;
    public DcMotor intake = null;
    public DcMotor shooter1 = null;
    public DcMotor shooter2 = null;
//    public CRServo lhs = null;
//
//    public DcMotor intake = null;
//
//    //servo motors
//    public Servo blocker = null;
//    public Servo ril = null;
//    public Servo lil = null;
//    public Servo claw = null;
//    public Servo ra = null;
//    public Servo la = null;
////    public DcMotor hanger = null;
//    //private CRServo robotLift = null;
//
//    public Servo backclaw = null;
//    //private CRServo robotLift = null;



    public controls_top_MainRobot(DcMotor Intake, DcMotor Shooter1, DcMotor Shooter2)

    {
        intake=Intake;
        shooter1= Shooter1;
        shooter2 = Shooter2;
//        ril = ri;
//        lil = li;
//        lsRight=lsR;
//        ra=r;
//        la=l;
//        lsLeft=lsL;
//        lhs = lhorizontal;
//        rhs = rhorizontal;
////        hanger = hang;
//        backclaw =bclaw;
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

    public void shooter1on()
    {
        shooter1.setPower(1);
    }
    public void shooter1half()
    {
        shooter1.setPower(0.65);
    }
    public void shooter1off()
    {
        shooter1.setPower(0);
    }
    public void shooter1reverse()
    {
        shooter1.setPower(-1);
    }


    public void shooter2on()
    {
        shooter2.setPower(1);
    }
    public void shooter2half()
    {
        shooter2.setPower(0.65);
    }
    public void shooter2off()
    {
        shooter2.setPower(0);
    }
    public void shooter2reverse()
    {
        shooter2.setPower(-1);
    }


}