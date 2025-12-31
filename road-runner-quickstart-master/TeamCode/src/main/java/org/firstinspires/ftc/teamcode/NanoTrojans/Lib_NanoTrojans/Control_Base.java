package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


//This class worked for road runner season 2024-2025
public class Control_Base {
    //private MecanumDrive drive;
    //private Drive drive;
    private BNO055IMU imu;

    private resources_base resourceBase;
    private DcMotor leftFront ;
    private DcMotor leftBack ;
    private DcMotor rightBack ;
    private DcMotor rightFront ;

    public Control_Base(HardwareMap hardwareMap)
    {

        resourceBase = new resources_base(hardwareMap);


        leftFront =resourceBase.leftFront;
        leftBack =resourceBase.leftBack;
        rightBack =resourceBase.rightBack;
        rightFront =resourceBase.rightFront;


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void driveRobot(double leftStickX, double leftStickY, double rightStickX) {

        double x = leftStickX;
        double y = -leftStickY; // Invert y to match coordinate system
        double turn = -rightStickX;

        // Get the current heading, taking into account the initial heading
        double initialHeading = 0;
        double currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - initialHeading;


        // Apply rotation matrix
        double rotatedX = x * Math.cos(-currentHeading) - y * Math.sin(-currentHeading);
        double rotatedY = x * Math.sin(-currentHeading) + y * Math.cos(-currentHeading);

        // Calculate motor speeds
        double frontLeftSpeed = rotatedX + rotatedY + turn;
        double frontRightSpeed = -rotatedX + rotatedY - turn;
        double backLeftSpeed = -rotatedX + rotatedY + turn;
        double backRightSpeed = rotatedX + rotatedY - turn;

        // Set motor powers, making sure they are in range
        leftFront.setPower(Math.max(-1.0, Math.min(frontLeftSpeed, 1.0)));
        rightFront.setPower(Math.max(-1.0, Math.min(frontRightSpeed, 1.0)));
        leftBack.setPower(Math.max(-1.0, Math.min(backLeftSpeed, 1.0)));
        rightBack.setPower(Math.max(-1.0, Math.min(backRightSpeed, 1.0)));

    }

}
