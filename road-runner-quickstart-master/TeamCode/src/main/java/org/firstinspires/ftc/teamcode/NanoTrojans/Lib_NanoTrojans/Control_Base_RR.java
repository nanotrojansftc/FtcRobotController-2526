package org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;




//This class worked for road runner season 2024-2025
public class Control_Base_RR {
    //private MecanumDrive drive;
    //private Drive drive;
    //private BNO055IMU imu;

//    private resources_base resourceBase;
//    private DcMotor leftFront ;
//    private DcMotor leftBack ;
//    private DcMotor rightBack ;
//    private DcMotor rightFront ;

    MecanumDrive drive;

    public Control_Base_RR(HardwareMap hardwareMap)
    {

//        resourceBase = new resources_base(hardwareMap);
//
//
//        leftFront =resourceBase.leftFront;
//        leftBack =resourceBase.leftBack;
//        rightBack =resourceBase.rightBack;
//        rightFront =resourceBase.rightFront;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);
       // MultipleTelemetry telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    public void driveRobot(double leftStickX, double leftStickY, double rightStickX) {
//           telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//            drive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            -leftStickY,
//                            -leftStickX
//                    ),
//                    -rightStickX
//
//            ));
//
//            drive.updatePoseEstimate();
//
////            telemetry.addData("x", drive.position.x);
////            telemetry.addData("y", drive.pose.position.y);
////            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
////            telemetry.update();
//
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.fieldOverlay().setStroke("#3F51B5");
//            //Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);


        //waitForStart();


            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -leftStickY,
                            -leftStickX
                    ),
                    -rightStickX
            ));

            drive.updatePoseEstimate();

            Pose2d pose = drive.localizer.getPose();
//            telemetry.addData("x", pose.position.x);
//            telemetry.addData("y", pose.position.y);
//            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
//            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

    }


