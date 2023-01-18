package org.firstinspires.ftc.teamcode.Tests;


// OpMode
        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Motors
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;

// IMU

// Other
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.Range;

        import org.openftc.easyopencv.OpenCvCamera;

// Constants / Other
        import org.firstinspires.ftc.teamcode.other.Constants;
        import org.firstinspires.ftc.teamcode.other.Threshold;


@TeleOp(name="Field Centric Drive", group="! TeleOp")

public class FieldCentricDrive extends LinearOpMode {

    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;


    private DcMotor Arm;
    private Servo Claw;

    private DcMotor TurnTable;

    private BNO055IMU imu;

    public static int ArmInterval = Constants.ArmConstants.armInterval;

    public static int maxArmPosition = Constants.ArmConstants.maxArmPosition;
    public static int lowArmPosition = Constants.ArmConstants.lowArmPosition;

    public static int highJuncArmPosition = Constants.ArmConstants.highJuncArmPosition;
    public static int midJuncArmPosition = Constants.ArmConstants.midJuncArmPosition;
    public static int lowJuncArmPosition = Constants.ArmConstants.lowJuncArmPosition;

    public static int turnTableFront = Constants.TurnTableConstants.TURNTABLE_FRONT;
    public static int turnTableLeft = Constants.TurnTableConstants.TURNTABLE_LEFT;
    public static int turnTableRight = Constants.TurnTableConstants.TURNTABLE_RIGHT;
    public static int turnTableBack = Constants.TurnTableConstants.TURNTABLE_BACK;

    public static int armThreshold = Constants.ArmConstants.armThreshold;


    int[] armSetPosition = {
            0,
            Constants.ArmConstants.lowArmPosition,
            Constants.ArmConstants.midJuncArmPosition,
            Constants.ArmConstants.highJuncArmPosition
    };

    boolean inArm = false;
    boolean lowArm = false;
    boolean midArm = false;
    boolean highArm = false;


    boolean isLowered = false;

    boolean upToggle = false;
    boolean leftToggle = false;
    boolean rightToggle = false;
    boolean downToggle = false;


    boolean resetPos = false;
    boolean firstPosReset = false;

    boolean leftPos = false;
    boolean rightPos = false;

    boolean topPos = false;

    float speedLimiter = Constants.DriveConstants.speedLimiter;


    double armPower = 1;
    double turnPower = .65;


    double clawOpenPosition = Constants.ClawConstants.openPosition;
    double clawClosePosition = Constants.ClawConstants.closePosition;

    @Override
    public void runOpMode() {

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Claw = hardwareMap.get(Servo.class, "Claw");
        TurnTable = hardwareMap.get(DcMotor.class, "TurnTable");

        imu = hardwareMap.get(BNO055IMU.class, "imu");


        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setPower(armPower);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TurnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurnTable.setTargetPosition(0);
        TurnTable.setPower(turnPower);
        TurnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        Arm.setDirection(DcMotorSimple.Direction.REVERSE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();
        while (opModeIsActive()) {


            double robotYaw = imu.getAngularOrientation().firstAngle;
            double robotRoll = imu.getAngularOrientation().secondAngle;
            double robotPitch = imu.getAngularOrientation().thirdAngle;

            if(Threshold.innerThresholdEqual(Math.toDegrees(robotRoll), 0, 3, 3)){
                robotRoll = 0;
            }
            if(Threshold.innerThresholdEqual(Math.toDegrees(robotPitch), 0, 5, 5)){
                robotPitch = 0;
            }


            double LY = -gamepad1.left_stick_y * -gamepad1.left_stick_y * -gamepad1.left_stick_y;
            double LX = gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x;
            double RX = gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x;

            double robotRotation = -imu.getAngularOrientation().firstAngle;

            double yValue = LX * Math.cos(robotRotation) - LY * Math.sin(robotRotation);
            double xValue = LX * Math.sin(robotRotation) + LY * Math.cos(robotRotation);

            if(Arm.getCurrentPosition() < armThreshold){
                speedLimiter = 1f;
            } else {
                speedLimiter = Constants.DriveConstants.speedLimiter;
            }

            Mecanum(xValue, yValue, RX, speedLimiter, robotRoll * 2, robotPitch * 2);


            // Arm Controls
            int calculatedArmPosition;

            calculatedArmPosition = Math.round((-gamepad2.left_stick_y * 400));

            if (Arm.getCurrentPosition() + calculatedArmPosition > maxArmPosition) {
                calculatedArmPosition = 0;
                Arm.setTargetPosition(maxArmPosition + 5);
            } else if (Arm.getCurrentPosition() + calculatedArmPosition < lowArmPosition) {
                calculatedArmPosition = 0;
                Arm.setTargetPosition(lowArmPosition - 5);
            } else {
                if (Math.abs(gamepad2.left_stick_y) > .025) {
                    if(Arm.getCurrentPosition() + calculatedArmPosition <= armThreshold && Threshold.outerThreshold(TurnTable.getCurrentPosition(), TurnTable.getTargetPosition(), -50, 50)){
                        int frontDistance = TurnTable.getCurrentPosition() - turnTableFront;
                        int leftDistance = Math.round(TurnTable.getCurrentPosition() - turnTableLeft);
                        int rightDistance = TurnTable.getCurrentPosition() - turnTableRight;
                        int backDistance = TurnTable.getCurrentPosition() - turnTableBack;

                        if(frontDistance < leftDistance && frontDistance < rightDistance && frontDistance < backDistance){
                            Arm.setTargetPosition(armThreshold);
                            Arm.setPower(armPower);
                            TurnTable.setTargetPosition(turnTableFront);
                            TurnTable.setPower(turnPower);
                        } else if (leftDistance < frontDistance && leftDistance < rightDistance && leftDistance < backDistance){
                            Arm.setTargetPosition(armThreshold);
                            Arm.setPower(armPower);
                            TurnTable.setTargetPosition(turnTableLeft);
                            TurnTable.setPower(turnPower);
                        } else if (rightDistance < frontDistance && rightDistance < leftDistance && rightDistance < backDistance){
                            Arm.setTargetPosition(armThreshold);
                            Arm.setPower(armPower);
                            TurnTable.setTargetPosition(rightDistance);
                            TurnTable.setPower(turnPower);
                        } else if (backDistance < frontDistance && backDistance < leftDistance && backDistance < rightDistance){
                            Arm.setTargetPosition(armThreshold);
                            Arm.setPower(armPower);
                            TurnTable.setTargetPosition(backDistance);
                            TurnTable.setPower(turnPower);
                        }
                    } else {
                        Arm.setTargetPosition(Arm.getCurrentPosition() + calculatedArmPosition);
                        Arm.setPower(Range.clip((Math.round(Math.abs(calculatedArmPosition)) / 400) + .25, -armPower, armPower));
                    }
                } else if (gamepad2.left_bumper) {
                    Arm.setTargetPosition(armSetPosition[1]);
                    Arm.setPower(armPower);
                } else if (gamepad2.right_bumper){
                    Arm.setTargetPosition(armSetPosition[2]);
                    Arm.setPower(armPower);
                } else {
                    //
                }
            }

            if(Threshold.innerThreshold(Arm.getTargetPosition(), armSetPosition[0], -200, 200)){
                inArm = true;
                lowArm = false;
                midArm = false;
                highArm = false;
            } else if(Threshold.innerThreshold(Arm.getTargetPosition(), armSetPosition[1], -200, 200)){
                lowArm = true;
                inArm = false;
                midArm = false;
                highArm = false;
            } else if(Threshold.innerThreshold(Arm.getTargetPosition(), armSetPosition[2], -200, 200)){
                midArm = true;
                inArm = false;
                lowArm = false;
                highArm = false;
            }


            if(Arm.getCurrentPosition() >= armThreshold - 25 && isLowered == true){
                if(upToggle == true){
                    TurnTable.setTargetPosition(turnTableFront);
                    TurnTable.setPower(turnPower);
                }
                if(leftToggle == true){
                    TurnTable.setTargetPosition(turnTableLeft);
                    TurnTable.setPower(turnPower);
                }
                if(rightToggle == true){
                    TurnTable.setTargetPosition(turnTableRight);
                    TurnTable.setPower(turnPower);
                }
                if(downToggle == true){
                    TurnTable.setTargetPosition(turnTableBack);
                    TurnTable.setPower(turnPower);
                }

                isLowered = false;

                upToggle = false;
                leftToggle = false;
                rightToggle = false;
                downToggle = false;
            } else if (-gamepad2.left_stick_y <= -.025){
                isLowered = false;

                upToggle = false;
                leftToggle = false;
                rightToggle = false;
                downToggle = false;
            }

            if(gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_down) {
                if (Arm.getCurrentPosition() < armThreshold - 20) {
                    Arm.setTargetPosition(armThreshold);
                    Arm.setPower(armPower);
                    isLowered = true;

                    if(gamepad2.dpad_up){
                        upToggle = true;
                    }
                    if(gamepad2.dpad_left){
                        leftToggle = true;
                    }
                    if(gamepad2.dpad_right){
                        rightToggle = true;
                    }
                    if(gamepad2.dpad_down){
                        downToggle = true;
                    }

                } else if (Arm.getCurrentPosition() > armThreshold - 20) {
                    if (gamepad2.dpad_up) {
                        TurnTable.setTargetPosition(turnTableFront);
                        TurnTable.setPower(turnPower);
                    }
                    if (gamepad2.dpad_left) {
                        TurnTable.setTargetPosition(turnTableLeft);
                        TurnTable.setPower(turnPower);
                    }
                    if (gamepad2.dpad_right) {
                        TurnTable.setTargetPosition(turnTableRight);
                        TurnTable.setPower(turnPower);
                    }
                    if (gamepad2.dpad_down) {
                        TurnTable.setTargetPosition(turnTableBack);
                        TurnTable.setPower(turnPower);
                    }

                }
            }


            // Claw Open/Close
            if(gamepad2.a){
                Claw.setPosition(clawClosePosition);
            }
            if(gamepad2.b){
                Claw.setPosition(clawOpenPosition);
            }

            if(gamepad2.y){
                topPos = true;
            }

            if(gamepad2.x){
                resetPos = true;
                firstPosReset = true;
            }
            if(gamepad2.left_bumper){
                leftPos = true;
            }

            if(gamepad2.right_bumper){
                rightPos = true;
            }


            if(resetPos) {
                if(Threshold.innerThreshold(TurnTable.getCurrentPosition(), turnTableBack, -50, 50)){
                    TurnTable.setTargetPosition(turnTableBack);
                    TurnTable.setPower(turnPower);

                    Arm.setTargetPosition(0);
                    Arm.setPower(armPower);
                    resetPos = false;
                } else {
                    if(Arm.getCurrentPosition() >= armThreshold){
                        TurnTable.setTargetPosition(turnTableBack);
                        TurnTable.setPower(turnPower);

                        Arm.setTargetPosition(armThreshold);
                        Arm.setPower(1);
                    }
                    if(Arm.getCurrentPosition() < armThreshold){
                        Arm.setTargetPosition(armThreshold);
                        Arm.setPower(1);
                    }
                }
            }

            if(leftPos) {
                if(Threshold.innerThreshold(TurnTable.getCurrentPosition(), turnTableBack, -50, 50)){
                    TurnTable.setTargetPosition(turnTableBack);
                    TurnTable.setPower(turnPower);

                    Arm.setTargetPosition(0);
                    Arm.setPower(armPower);
                    leftPos = false;
                } else {
                    if(Arm.getCurrentPosition() >= armThreshold){
                        TurnTable.setTargetPosition(turnTableLeft);
                        TurnTable.setPower(turnPower);

                        Arm.setTargetPosition(armThreshold);
                        Arm.setPower(1);
                    }
                    if(Arm.getCurrentPosition() < armThreshold){
                        Arm.setTargetPosition(armThreshold);
                        Arm.setPower(1);
                    }
                }
            }

            if(rightPos) {
                if(Threshold.innerThreshold(TurnTable.getCurrentPosition(), turnTableBack, -50, 50)){
                    TurnTable.setTargetPosition(turnTableRight);
                    TurnTable.setPower(turnPower);

                    Arm.setTargetPosition(0);
                    Arm.setPower(armPower);
                    rightPos = false;
                } else {
                    if(Arm.getCurrentPosition() >= armThreshold){
                        TurnTable.setTargetPosition(turnTableBack);
                        TurnTable.setPower(turnPower);

                        Arm.setTargetPosition(armThreshold);
                        Arm.setPower(1);
                    }
                    if(Arm.getCurrentPosition() < armThreshold){
                        Arm.setTargetPosition(armThreshold);
                        Arm.setPower(1);
                    }
                }
            }


            if(topPos){
                Arm.setTargetPosition(highJuncArmPosition);
                Arm.setPower(1);

                if(Arm.getCurrentPosition() >= armThreshold){
                    TurnTable.setTargetPosition(turnTableFront);
                    TurnTable.setPower(turnPower);
                    topPos = false;
                }
            }


            telemetry.addLine("Gamepad 1");
            telemetry.addData("Left Stick Y", -gamepad1.left_stick_y);
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Right Stick X", gamepad1.right_stick_x);

            double gamepad1Hypotenuse = Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            double gamepad1LeftDirection = (Math.cos(-gamepad1.left_stick_y / gamepad1Hypotenuse));
            telemetry.addData("Direction (deg)", gamepad1Hypotenuse);
            telemetry.addData("Rotation (deg)", 180 * gamepad1.right_stick_x);

            telemetry.addLine();
            telemetry.addLine("Gamepad 2");
            telemetry.addData("Left Stick Y", -gamepad2.left_stick_y);
            telemetry.addData("Left Stick X", gamepad2.left_stick_x);
            telemetry.addData("Right Stick X", gamepad2.right_stick_x);

            telemetry.addLine();
            telemetry.addLine("Motors");
            telemetry.addData("Front Left", FrontLeft.getCurrentPosition());
            telemetry.addData("Back Right", BackRight.getCurrentPosition());

            telemetry.addLine();
            telemetry.addLine("Turn Table");
            telemetry.addData("Turn Table Set Position", TurnTable.getTargetPosition());
            String turnTablePos = "IN MOVEMENT, ";
            if(Threshold.innerThreshold(TurnTable.getCurrentPosition(), turnTableFront, -50, 50)){
                turnTablePos = "FRONT, ";
            } else if(Threshold.innerThreshold(TurnTable.getCurrentPosition(), turnTableLeft, -50, 50)){
                turnTablePos = "LEFT, ";
            } else if(Threshold.innerThreshold(TurnTable.getCurrentPosition(), turnTableRight, -50, 50)){
                turnTablePos = "RIGHT, ";
            } else if(Threshold.innerThreshold(TurnTable.getCurrentPosition(), turnTableBack, -50, 50)){
                turnTablePos = "BACK, ";
            }
            telemetry.addData("Turn Table Position", turnTablePos + TurnTable.getCurrentPosition());

            telemetry.addLine();
            telemetry.addLine("Arm");
            telemetry.addData("Arm Set Position", Arm.getTargetPosition());
            telemetry.addData("Arm Position", Arm.getCurrentPosition());

            telemetry.addLine();
            telemetry.addData("Claw Position", Claw.getPosition());

            telemetry.addLine();
            telemetry.addLine("Robot Rotations");
            telemetry.addData("Robot Rotation (Yaw)", imu.getAngularOrientation().firstAngle);
            telemetry.addData("Robot Roll", imu.getAngularOrientation().secondAngle);
            telemetry.addData("Robot Pitch", imu.getAngularOrientation().thirdAngle);

            telemetry.addData("inArm", inArm);
            telemetry.addData("lowArm", lowArm);
            telemetry.addData("midArm", midArm);
            telemetry.addData("highArm", highArm);
            telemetry.update();
        }
    }

    // Mecanum Drive
    private void Mecanum(double LY, double LX, double RX, float speedLimiter, double robotRoll, double robotPitch){
        double limiter = Math.max(Math.abs(LY) + Math.abs(LX) + Math.abs(RX), 1);

        FrontLeft.setPower((((LY + LX + RX)) / limiter) / speedLimiter + robotPitch - robotRoll);
        FrontRight.setPower((((LY - LX - RX)) / limiter) / speedLimiter + robotPitch + robotRoll);
        BackLeft.setPower((((LY - LX + RX)) / limiter) / speedLimiter + robotPitch + robotRoll);
        BackRight.setPower((((LY + LX - RX)) / limiter) / speedLimiter + robotPitch - robotRoll);
    }
}


