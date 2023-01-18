package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="ParkAuto", group="! RegionalAutonomous")
public class ParkAuto extends LinearOpMode{

    // Chassis Wheels
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    // Build Features
    private DcMotor Arm;
    private Servo Claw;

    // Phone Camera
    OpenCvCamera phoneCam;

    public static int ParkLoca;



    @Override
    public void runOpMode() {

        // Motors

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Claw = hardwareMap.get(Servo.class, "Claw");


        // Wheels

        // FrontLeft
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // FrontRight
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // BackLeft
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // BackRight
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Build Features

        // Arm
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Arm Init
        Arm.setPower(1);
        Arm.setTargetPosition(0);



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, 70, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

/*      Trajectory format & displacementMarker

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .build();

        .addDisplacementMarker(0, () -> {
        })
*/
        // Third parking location
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(28)
                .build();


        // Second parking location
        Trajectory P2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(2)
                .build();


        while (!isStarted()) {

            waitForStart();
            if (isStopRequested())
                return;

            drive.followTrajectory(traj1);
            drive.followTrajectory(P2);

//            switch (detector.getLocation()) {
//                case P1:
//                    ParkLoca = 1;
//                    telemetry.addData("Parking", "1");
//                    break;
//                case P2:
//                    ParkLoca = 2;
//                    telemetry.addData("Parking", "2");
//                    break;
//                case P3:
//                    ParkLoca = 3;
//                    telemetry.addData("Parking", "3");
//                    break                                              ;
//                case NOT_FOUND:
//                    ParkLoca = 1;
//                    telemetry.addData("Parking", "NOT FOUND");
//            }--

            telemetry.update();
        }
    }

    private void ClawOpen(){
        Claw.setPosition(.65);
    }

    private void ClawClose(){
        Claw.setPosition(0);
    }

    private void ClawReset(){
        Claw.setPosition(.65);
    }

    private void ArmTop(){
        Arm.setTargetPosition(2570);
        Arm.setPower(1);
    }
    private void ArmMid(){
        Arm.setTargetPosition(1825);
        Arm.setPower(1);
    }
    private void ArmBot(){
        Arm.setTargetPosition(1090);
        Arm.setPower(1);
    }
    private void ArmReset(){
        Arm.setPower(1);
        Arm.setTargetPosition(0);
    }

    private void ArmSetPos(int x){
        Arm.setPower(1);
        Arm.setTargetPosition(x);
    }
}
