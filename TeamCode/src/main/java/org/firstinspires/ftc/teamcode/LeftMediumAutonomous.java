package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.other.Constants;
import org.firstinspires.ftc.teamcode.other.Threshold;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="Left Medium Autonomous", group="! State", preselectTeleOp="State TeleOp")
public class LeftMediumAutonomous extends LinearOpMode {

    // Chassis Wheels
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    // Build Features
    private DcMotor TurnTable;
    private DcMotor Arm;
    private Servo Claw;


    private BNO055IMU imu;


    // Phone Camera
    OpenCvWebcam webcam;

    public int ParkLoca;


    public static double endingRotation;
    public static int endingArmPosition;
    public static int endingTurnTablePosition;

    boolean turnTableResetting = true;
    boolean firstCheck = true;

    ElapsedTime resetTurnTable = new ElapsedTime();


    @Override
    public void runOpMode() {

        // Motors

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        TurnTable = hardwareMap.get(DcMotor.class, "TurnTable");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Claw = hardwareMap.get(Servo.class, "Claw");

        imu = hardwareMap.get(BNO055IMU.class, "imu");


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

        TurnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while(turnTableResetting){

            TurnTable.setPower(0.25);

            if(resetTurnTable.milliseconds() >= 1000) {
                int turnTableOffset = 90;

                if(firstCheck){
                    TurnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    TurnTable.setTargetPosition(0);
                    TurnTable.setPower(1);
                    TurnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    firstCheck = false;
                }

                TurnTable.setTargetPosition(-turnTableOffset);
                TurnTable.setPower(1);

                System.out.println("Setting");
                System.out.println(TurnTable.getCurrentPosition());

                if(Threshold.innerThreshold(TurnTable.getCurrentPosition(), TurnTable.getTargetPosition(), -5, 5)){
                    turnTableResetting = false;
                }
            }
        }


        // Arm
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setPower(1);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        TurnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurnTable.setTargetPosition(0);
        TurnTable.setPower(1);
        TurnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Arm.setDirection(DcMotorSimple.Direction.REVERSE);


        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        SleeveDetector detector = new SleeveDetector(telemetry);
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(30.5, 64.5, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

/*      Trajectory format & displacementMarker

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .build();

        .addDisplacementMarker(0, () -> {
        })
*/
        int cycleArmPosition[] = {560, 425, 290, 155, 0};

        final int TURNTABLE_FRONT = Constants.TurnTableConstants.TURNTABLE_FRONT;
        final int TURNTABLE_LEFT = Constants.TurnTableConstants.TURNTABLE_LEFT;
        final int TURNTABLE_RIGHT = Constants.TurnTableConstants.TURNTABLE_RIGHT;
        final int TURNTABLE_BACK = Constants.TurnTableConstants.TURNTABLE_BACK;

        float placeX = 25f;
        float placeY = 13f;

        float placeOffset = 1f;
        float turnOffset = 1;
        float armOffset = 12;

        float stackX = 59.5f;
        float stackY = 12.25f;



        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(36, 50), Math.toRadians(-90))
                .addDisplacementMarker(0, () -> {
                    ClawClose();
                })
                .addDisplacementMarker(4, () -> {
                    ArmMid(1);
                })
                .splineToSplineHeading(new Pose2d(35.5, 23, Math.toRadians(-180)), Math.toRadians(-90))
                .addSpatialMarker(new Vector2d(35.5, 23 + placeOffset), () -> {
                    ClawOpen();
                    sleep(45);
                })
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .addDisplacementMarker(turnOffset, () -> {
                    TurnTablePos(TURNTABLE_FRONT, 0.3);
                })
                .addDisplacementMarker(armOffset, () -> {
                    ArmSetPos(cycleArmPosition[0], 1);
                })
                .splineToConstantHeading(new Vector2d(40, 15), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(stackX, stackY, Math.toRadians(0)), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(stackX, stackY), () -> {
                    ClawClose();
                    sleep(100);
                })
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .addDisplacementMarker(.25, () -> {
                    ArmMid(.75);
                })
                .lineToSplineHeading(new Pose2d(placeX, placeY - .075, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(50, 10), () -> {
                    TurnTablePos(TURNTABLE_LEFT, 0.2);
                })
                .addSpatialMarker(new Vector2d(placeX + placeOffset, placeY), () -> {
                    ClawOpen();
                    sleep(125);
                })
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .addDisplacementMarker(turnOffset, () -> {
                    TurnTablePos(TURNTABLE_FRONT, 0.3);
                })
                .addDisplacementMarker(armOffset, () -> {
                    ArmSetPos(cycleArmPosition[1], 1);
                })
                .lineToSplineHeading(new Pose2d(stackX, stackY, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(stackX, stackY), () -> {
                    ClawClose();
                    sleep(100);
                })
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .addDisplacementMarker(.25, () -> {
                    ArmMid(.75);
                })
                .lineToSplineHeading(new Pose2d(placeX, placeY - .15, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(50, 10), () -> {
                    TurnTablePos(TURNTABLE_LEFT, 0.2);
                })
                .addSpatialMarker(new Vector2d(placeX + placeOffset, placeY), () -> {
                    ClawOpen();
                    sleep(125);
                })
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .addDisplacementMarker(turnOffset, () -> {
                    TurnTablePos(TURNTABLE_FRONT, 0.3);
                })
                .addDisplacementMarker(armOffset, () -> {
                    ArmSetPos(cycleArmPosition[2], 1);
                })
                .lineToSplineHeading(new Pose2d(stackX, stackY, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(stackX, stackY), () -> {
                    ClawClose();
                    sleep(100);
                })
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .addDisplacementMarker(.25, () -> {
                    ArmMid(.75);
                })
                .lineToSplineHeading(new Pose2d(placeX + .25, placeY - .5, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(50, 10), () -> {
                    TurnTablePos(TURNTABLE_LEFT, .2);
                })
                .addSpatialMarker(new Vector2d(placeX + placeOffset, placeY), () -> {
                    ClawOpen();
                    sleep(75);
                })
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .addDisplacementMarker(turnOffset, () -> {
                    TurnTablePos(TURNTABLE_FRONT, 0.275);
                })
                .addDisplacementMarker(armOffset, () -> {
                    ArmSetPos(cycleArmPosition[3], 1);
                })
                .lineToSplineHeading(new Pose2d(stackX, stackY, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(stackX, stackY), () -> {
                    ClawClose();
                    sleep(125);
                })
                .build();

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .addDisplacementMarker(.25, () -> {
                    ArmMid(.75);
                })
                .lineToSplineHeading(new Pose2d(placeX - .25, placeY - .5, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(50, 10), () -> {
                    TurnTablePos(TURNTABLE_LEFT, 0.2);
                })
                .addSpatialMarker(new Vector2d(placeX + placeOffset, placeY), () -> {
                    ClawOpen();
                    sleep(75);
                })
                .build();

        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .addDisplacementMarker(turnOffset, () -> {
                    TurnTablePos(TURNTABLE_FRONT, 0.3);
                })
                .addDisplacementMarker(armOffset, () -> {
                    ArmSetPos(cycleArmPosition[4], 1);
                })
                .lineToSplineHeading(new Pose2d(stackX, stackY, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(stackX, stackY), () -> {
                    ClawClose();
                    sleep(125);
                })
                .build();

        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .addDisplacementMarker(.5, () -> {
                    ArmMid(.75);
                })
                .lineToSplineHeading(new Pose2d(placeX - .25, placeY - .6, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(50, 10), () -> {
                    TurnTablePos(TURNTABLE_LEFT, 0.2);
                })
                .addSpatialMarker(new Vector2d(placeX + placeOffset, placeY), () -> {
                    ClawOpen();
                    sleep(75);
                })
                .build();


        Trajectory P1 = drive.trajectoryBuilder(traj11.end())
                .addDisplacementMarker(4, () -> {
                    TurnTablePos(TURNTABLE_FRONT, .25);
                    ArmReset(1);
                })
                .forward(32)
                .build();

        Trajectory P2 = drive.trajectoryBuilder(traj11.end())
                .addDisplacementMarker(4, () -> {
                    TurnTablePos(TURNTABLE_FRONT, .25);
                    ArmReset(1);
                })
                .forward(11)
                .build();

        Trajectory P3 = drive.trajectoryBuilder(traj11.end())
                .addDisplacementMarker(4, () -> {
                    TurnTablePos(TURNTABLE_FRONT, .25);
                    ArmReset(1);
                })
                .back(14)
                .build();


        telemetry.addLine("Ready to Start!");
        telemetry.update();



        waitForStart();
        if (isStopRequested())
            return;

        switch (detector.getLocation()) {
            case P1:
                ParkLoca = 1;
                telemetry.addData("Parking", "1");
                break;
            case P2:
                ParkLoca = 2;
                telemetry.addData("Parking", "2");
                break;
            case P3:
                ParkLoca = 3;
                telemetry.addData("Parking", "3");
                break;
            case NOT_FOUND:
                ParkLoca = 1;
                telemetry.addData("Parking", "NOT FOUND");
        }


        webcam.stopStreaming();
        telemetry.update();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
        drive.followTrajectory(traj10);
        drive.followTrajectory(traj11);

        // Parking Locations
        if (ParkLoca == 1) {
            drive.followTrajectory(P1);
        } else if (ParkLoca == 2) {
            drive.followTrajectory(P2);
        } else if (ParkLoca == 3) {
            drive.followTrajectory(P3);
        } else {
            drive.followTrajectory(P1);
        }


        sleep(1000);
    }


    private void ClawOpen(){
        Claw.setPosition(Constants.ClawConstants.openPosition);
    }

    private void ClawClose(){
        Claw.setPosition(Constants.ClawConstants.closePosition);
    }

    private void ClawReset(){
        Claw.setPosition(Constants.ClawConstants.openPosition);
    }

    private void ArmTop(double power){
        Arm.setTargetPosition(Constants.ArmConstants.highJuncArmPosition);
        Arm.setPower(power);
    }
    private void ArmMid(double power){
        Arm.setTargetPosition(Constants.ArmConstants.midJuncArmPosition);
        Arm.setPower(power);
    }
    private void ArmBot(double power){
        Arm.setTargetPosition(Constants.ArmConstants.lowArmPosition);
        Arm.setPower(power);
    }

    private void ArmReset(double power){
        Arm.setPower(power);
        Arm.setTargetPosition(0);
    }

    private void ArmSetPos(int x, double power){
        Arm.setPower(1);
        Arm.setTargetPosition(x);
    }

    int armThreshold = Constants.ArmConstants.armThreshold;

    private void TurnTablePos(int setPosition, double speed) {
        TurnTable.setTargetPosition(setPosition);
        TurnTable.setPower(speed);
    }
}
