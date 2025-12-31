/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.NanoTrojans.Auto_NanoTrojans;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.NanoTrojans.Lib_NanoTrojans.resources_base;



/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name = "Auto_Encoder_Full_Test")
public class Auto_Encoder_Full_Test extends LinearOpMode {


    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    private resources_base resourceBase;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        resourceBase = new resources_base(hardwareMap);

        frontLeftMotor = resourceBase.leftFront;
        frontRightMotor = resourceBase.rightFront;
        rearLeftMotor = resourceBase.leftBack;
        rearRightMotor = resourceBase.rightBack;

        // Set motor modes
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
//        drive.followTrajectory(trajectory);
        boolean stop = false;
        while (opModeIsActive() && !stop) {

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            Run2seconds( 0.3);
            //strafeRight(48, 1);


            stop = true;
        }

    }


    private void setRunMode(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        rearLeftMotor.setMode(mode);
        rearRightMotor.setMode(mode);
    }


    private void Run2seconds(double power) {

        telemetry.addLine("Front Right running");
        frontRightMotor.setPower(power);
        sleep(3000);
        frontRightMotor.setPower(0);

        telemetry.addLine("Front Left running");
        frontLeftMotor.setPower(power);
        sleep(3000);
        frontLeftMotor.setPower(0);

        telemetry.addLine("Rear Right running");
        rearRightMotor.setPower(power);
        sleep(3000);
        rearRightMotor.setPower(0);


        telemetry.addLine("Rear Left running");
        rearLeftMotor.setPower(power);
        sleep(3000);
        rearLeftMotor.setPower(0);

        telemetry.addLine("Front Right and Left running");
        frontRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(power);
        sleep(3000);


        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);


    }


}
