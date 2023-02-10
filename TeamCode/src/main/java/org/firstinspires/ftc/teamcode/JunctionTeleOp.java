package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.other.Threshold;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="Junction TeleOp", group="! State")
public class JunctionTeleOp extends LinearOpMode {

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


    public static int turnTablePos = 0;
    public static int turnTableTargetPos = 0;

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


        JunctionTrack detector = new JunctionTrack(telemetry);
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
        waitForStart();

        while (opModeIsActive()){
            turnTablePos = TurnTable.getCurrentPosition();
            TurnTable.setTargetPosition(JunctionTrack.targetPosition);
            TurnTable.setPower(.4);
        }
    }
}
