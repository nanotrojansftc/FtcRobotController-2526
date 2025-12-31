package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class controls_top {
    // Constants for encoder counts and wheel measurements
    static final double COUNTS_PER_REVOLUTION = 537.7; // Encoder counts per revolution
    static final double WHEEL_DIAMETER_MM = 96.0; // Wheel diameter in millimeters
    static final double MM_PER_REVOLUTION = WHEEL_DIAMETER_MM * Math.PI; // Wheel circumference
    static final double COUNTS_PER_MM = COUNTS_PER_REVOLUTION / MM_PER_REVOLUTION; // Counts per millimeter
    static final double COUNTS_PER_INCH = COUNTS_PER_MM * 25.4; // Counts per inch

    //private DcMotor intake = null;
    public DcMotor lgun = null;
    public DcMotor rgun = null;
    public DcMotor intake = null;

    public Servo llift = null;
   public Servo rlift = null;
    public CRServo fspin = null;
    public CRServo rspin = null;
    public CRServo lspin = null;
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



    public controls_top(DcMotor lgun1, DcMotor rgun1,  DcMotor intake1, Servo llift1, Servo rlift1, CRServo fspin1, CRServo rspin1, CRServo lspin1)

    {
        lgun = lgun1;
        rgun = rgun1;
        intake=intake1;
        llift = llift1;
        rlift = rlift1;
        fspin = fspin1;
        rspin = rspin1;
        lspin = lspin1;

    }

    public void  shootleftStart()
    {
        lgun.setPower(1);
    }

    public void shootleftStop()
    {
        lgun.setPower(0);
    }

    public void leftliftUp()
    {

        llift.setPosition(0.625);
    }


    public void leftliftDown()
    {
        llift.setPosition(1);
    }


    public void shootrightStart()
    {
        rgun.setPower(-1);

    }

    public void shootrightStop()
    {
        rgun.setPower(0);
    }

    public void rihtliftUp()
    {

        rlift.setPosition(0.3);

    }

    public void rihtliftDown()
    {

        rlift.setPosition(0.005);

    }

//    private void rotate()
//    {
//        if (carousel ==2){
//            carousel =0;
//            resources.rspin.setPower(1);
//            sleep(475);
//            resources.rspin.setPower(0);
//        }
//        else if (carousel ==1){
//            carousel +=1;
//            resources.lspin.setPower(1);
//            sleep(475);
//            resources.lspin.setPower(0);
//        }
//        else if (carousel ==0){
//            carousel +=1;
//            resources.fspin.setPower(1);
//            sleep(475);
//            resources.fspin.setPower(0);
//        }
//    }

//    private void shootgreen()
//    {
//        if(bench.getDetectedColor(telemetry) == colorsensors.DetectedColor.GREEN)
//            shootleft();
//        else if(bench.getrightcolor(telemetry) == colorsensors.rightcolor.GREEN)
//            shootright();
//        else   //rotate
//        {
//            //rotate();
//            //sleep(200);
//        }
//    }
//
//    private void shootpurple()
//    {
//        if(bench.getDetectedColor(telemetry) == colorsensors.DetectedColor.PURPLE)
//            shootleft();
//        else if(bench.getrightcolor(telemetry) == colorsensors.rightcolor.PURPLE)
//            shootright();
//        else   //rotate
//        {
//            //rotate();
//            //sleep(200);
//        }
//    }

}