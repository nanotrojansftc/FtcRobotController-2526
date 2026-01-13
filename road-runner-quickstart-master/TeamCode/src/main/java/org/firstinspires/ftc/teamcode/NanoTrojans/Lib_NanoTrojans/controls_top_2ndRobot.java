package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class controls_top_2ndRobot {
    // Constants for encoder counts and wheel measurements
    static final double COUNTS_PER_REVOLUTION = 537.7; // Encoder counts per revolution
    static final double WHEEL_DIAMETER_MM = 96.0; // Wheel diameter in millimeters
    static final double MM_PER_REVOLUTION = WHEEL_DIAMETER_MM * Math.PI; // Wheel circumference
    static final double COUNTS_PER_MM = COUNTS_PER_REVOLUTION / MM_PER_REVOLUTION; // Counts per millimeter
    static final double COUNTS_PER_INCH = COUNTS_PER_MM * 25.4; // Counts per inch

    //private DcMotor intake = null;

    public DcMotor shooter = null;
    public DcMotor intake = null;

    public Servo lift1 = null;
   public Servo lift2 = null;
    public Servo lift3 = null;


    public controls_top_2ndRobot(DcMotor shooter1,  DcMotor intake1, Servo lift1a, Servo lift2a, Servo lift3a)

    {

        shooter = shooter1;
        intake=intake1;
        lift1 = lift1a;
        lift2=  lift2a;
        lift3=  lift2a;

    }

    public void  shootStart()
    {
        shooter.setPower(1);
    }

    public void shootStop()
    {
        shooter.setPower(0);
    }

    public void  intakeStart()
    {
        intake.setPower(1);
    }

    public void intakeStop()
    {
        intake.setPower(0);
    }

    public void lift1Up()
    {
        lift1.setPosition(0.625);
    }

    public void lift1Down()
    {
        lift1.setPosition(1);
    }

    public void lift2Up()
    {
        lift2.setPosition(0.625);
    }

    public void lift2Down()
    {
        lift2.setPosition(1);
    }

    public void lift3Up()
    {
        lift3.setPosition(0.625);
    }

    public void lift3Down()
    {
        lift3.setPosition(1);
    }


}