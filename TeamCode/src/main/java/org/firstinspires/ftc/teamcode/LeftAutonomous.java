package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.other.Constants;
import org.firstinspires.ftc.teamcode.other.Threshold;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="Left Autonomous", group="! State", preselectTeleOp="State TeleOp")
public class LeftAutonomous extends LinearOpMode {

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
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(34, 65.5, Math.toRadians(-90));
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


        float stackDistance = 56.45f;
        float placeDistance = 24f;

        Trajectory traj1 = drive.trajectoryBuilder(startPose)

                .build();



        waitForStart();
        if (isStopRequested())
            return;

        drive.followTrajectory(traj1);
    }
}
