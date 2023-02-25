package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.other.PIDController;

@Config
@TeleOp(name = "PIDControllerTesting", group = "Testing")
public class PIDControllerTesting extends LinearOpMode {
    private DcMotorEx FrontLeft;


    PIDController FrontLeftController = new PIDController();

    public static double Kp;
    public static double Ki;
    public static double Kd;


    public static double targetPosition;

    public void runOpMode(){

        FrontLeft = hardwareMap.get(DcMotorEx.class, "Arm");

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while(opModeIsActive()){
            FrontLeftController.setPIDCoefficents(Kp, Ki, Kd);
            FrontLeftController.updatePIDController(targetPosition, FrontLeft.getCurrentPosition());
            FrontLeft.setVelocity(FrontLeftController.getOutput());


            telemetry.addData("FrontLeft Position", FrontLeft.getCurrentPosition());
            telemetry.addData("FrontLeft Target Position", targetPosition);
            telemetry.addData("Error", FrontLeft.getCurrentPosition() - targetPosition);
            telemetry.addData("FrontLeft Velocity", FrontLeft.getVelocity());
            telemetry.update();
        }
    }
}
