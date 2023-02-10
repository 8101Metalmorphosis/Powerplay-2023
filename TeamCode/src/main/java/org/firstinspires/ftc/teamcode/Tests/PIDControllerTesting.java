package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PIDControllerTesting extends LinearOpMode {
    private DcMotorEx FrontLeft;


    public void runOpMode(){

        FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while(opModeIsActive()){

        }
    }
}
