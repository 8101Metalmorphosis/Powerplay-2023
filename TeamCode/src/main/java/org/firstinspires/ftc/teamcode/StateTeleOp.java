package org.firstinspires.ftc.teamcode;

// OpMode
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Motors
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// IMU

// Other
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Constants / Other
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.other.Constants;
import org.firstinspires.ftc.teamcode.other.Threshold;


@TeleOp(name="State TeleOp", group="! State")

public class StateTeleOp extends LinearOpMode {

    private DcMotorEx FrontLeft;
    private DcMotorEx FrontRight;
    private DcMotorEx BackLeft;
    private DcMotorEx BackRight;


    private DcMotorEx Arm;
    private DcMotorEx TurnTable;

    private Servo Claw;

    private BNO055IMU imu;


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
    public static int armInterval = Constants.ArmConstants.armInterval;



    boolean inArm = false;
    boolean lowArm = false;
    boolean midArm = false;
    boolean highArm = false;


    boolean isLowered = false;

    boolean upToggle = false;
    boolean leftToggle = false;
    boolean rightToggle = false;
    boolean downToggle = false;

    // TurnTable Macros
    boolean leftMacro = false;
    boolean rightMacro = false;

    boolean antiTip = true;

    boolean clawToggle = false;

    boolean override = false;
    boolean firstToggle = false;


    float speedLimiter = Constants.DriveConstants.speedLimiter;


    double armPower = 1;
    double turnPower = .65;


    double clawOpenPosition = Constants.ClawConstants.openPosition;
    double clawClosePosition = Constants.ClawConstants.closePosition;


    int endGameTime = 85_000;
    int parkTime = 115_000;


    ElapsedTime overrideTimer = new ElapsedTime();




    @Override
    public void runOpMode() {

        FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        Arm = hardwareMap.get(DcMotorEx.class, "Arm");
        Claw = hardwareMap.get(Servo.class, "Claw");
        TurnTable = hardwareMap.get(DcMotorEx.class, "TurnTable");

        imu = hardwareMap.get(BNO055IMU.class, "imu");


        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        TurnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurnTable.setTargetPosition(0);
        TurnTable.setPower(turnPower);
        TurnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setPower(armPower);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        Arm.setDirection(DcMotorSimple.Direction.REVERSE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        if(Threshold.innerThreshold(Arm.getCurrentPosition(), Arm.getTargetPosition(), -5, 5)){
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm.setTargetPosition(0);
            Arm.setPower(armPower);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        ElapsedTime endGameTimer = new ElapsedTime();
        ElapsedTime speedTimer = new ElapsedTime();

        gamepad1.setLedColor(0.25, 0.1, 0.6, 100000);
        gamepad2.setLedColor(0.25, 0.1, 0.6, 100000);

        while (opModeIsActive()) {

            if(Threshold.innerThreshold(endGameTimer.milliseconds(), endGameTime, 0, 1000)){
                if(Threshold.innerThreshold(endGameTimer.milliseconds(), endGameTime, 0, 250)){
                    gamepad1.rumble(100);
                    gamepad2.rumble(100);
                } else if (Threshold.innerThreshold(endGameTimer.milliseconds(), endGameTime, 500, 750)){
                    gamepad1.rumble(100);
                    gamepad2.rumble(100);
                }
            } else if(Threshold.innerThreshold(endGameTimer.milliseconds(), parkTime, 0, 1000)){
                if(Threshold.innerThreshold(endGameTimer.milliseconds(), parkTime, 0, 250)){
                    gamepad1.rumble(100);
                    gamepad2.rumble(100);
                } else if (Threshold.innerThreshold(endGameTimer.milliseconds(), parkTime, 500, 750)){
                    gamepad1.rumble(100);
                    gamepad2.rumble(100);
                }
            } else {
                gamepad1.stopRumble();
                gamepad2.stopRumble();
            }

            // Mecanum Inputs
            double LY = -gamepad1.left_stick_y;
            double LX = gamepad1.left_stick_x;
            double RX = gamepad1.right_stick_x;

            double robotYaw = imu.getAngularOrientation().firstAngle;
            double robotRoll = imu.getAngularOrientation().secondAngle;
            double robotPitch = imu.getAngularOrientation().thirdAngle;


            if (Threshold.innerThresholdEqual(Math.toDegrees(robotRoll), 0, 3, 3)) {
                robotRoll = 0;
            }
            if (Threshold.innerThresholdEqual(Math.toDegrees(robotPitch), 0, 5, 5)) {
                robotPitch = 0;
            }

            if(gamepad1.options){
                antiTip = false;
            }

            if(!antiTip){
                robotPitch = 0;
                robotRoll = 0;
            }

            if(gamepad1.a && speedTimer.milliseconds() >= 100){
                if(speedLimiter == 1.2){
                    speedLimiter = 1.5f;
                } else if (speedLimiter == 1.5){
                    speedLimiter = 1.2f;
                }
                speedTimer = new ElapsedTime();
            }

            Mecanum(LY, LX, RX, speedLimiter, robotRoll * 3, robotPitch * 3);


            if(gamepad2.share && gamepad2.options && overrideTimer.milliseconds() >= 250){
                override = !override;
                firstToggle = true;
                overrideTimer = new ElapsedTime();
            }

            if(override){

                if(firstToggle){
                    if(Claw.getPosition() == clawClosePosition){
                        Claw.setPosition(clawOpenPosition);
                    } else {
                        Claw.setPosition(clawClosePosition);
                    }
                    Arm.setPower(0);
                    TurnTable.setPower(0);

                    Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    TurnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    TurnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    firstToggle = false;
                }


                Arm.setPower(-gamepad2.left_stick_y);
                TurnTable.setPower(gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x);

                // Claw Controls
                if (gamepad2.b) {
                    Claw.setPosition(clawOpenPosition);
                }

                if (gamepad2.a) {
                    Claw.setPosition(clawClosePosition);
                }

            } else {
                if(firstToggle){
                    Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    TurnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    TurnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    TurnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    firstToggle = false;
                }

                // Manual Arm Controls
                int calculatedArmPosition;

                calculatedArmPosition = Math.round((-gamepad2.left_stick_y * armInterval));

                if (Arm.getCurrentPosition() + calculatedArmPosition > maxArmPosition) {
                    calculatedArmPosition = 0;
                    Arm.setTargetPosition(maxArmPosition + 5);
                } else if (Arm.getCurrentPosition() + calculatedArmPosition < lowArmPosition) {
                    calculatedArmPosition = 0;
                    Arm.setTargetPosition(lowArmPosition - 5);
                } else {
                    if (Math.abs(gamepad2.left_stick_y) > .025) {
                        Arm.setTargetPosition(Arm.getCurrentPosition() + calculatedArmPosition);
                        Arm.setPower(Range.clip((Math.round(Math.abs(calculatedArmPosition)) / armInterval) + .25, 0, 1));
                    }
                }


                // Automated Arm Controls
                if (gamepad2.right_bumper) {
                    Arm.setTargetPosition(highJuncArmPosition);
                    Arm.setPower(1);
                }

                if (gamepad2.right_trigger >= .025) {
                    Arm.setTargetPosition(midJuncArmPosition);
                    Arm.setPower(1);
                }

                if (gamepad2.left_bumper) {
                    Arm.setTargetPosition(lowJuncArmPosition);
                    Arm.setPower(1);
                }

                if (gamepad2.left_trigger >= .025) {
                    Arm.setTargetPosition(0);
                    Arm.setPower(1);
                }

                // TurnTable Controls
                if (gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_down) {
                    if (Arm.getCurrentPosition() < armThreshold) {
                        Arm.setTargetPosition(armThreshold + 50);
                        Arm.setPower(armPower);
                        isLowered = true;

                        if (gamepad2.dpad_up) {
                            upToggle = true;
                        }
                        if (gamepad2.dpad_left) {
                            leftToggle = true;
                        }
                        if (gamepad2.dpad_right) {
                            rightToggle = true;
                        }
                        if (gamepad2.dpad_down) {
                            downToggle = true;
                        }

                    } else if (Arm.getCurrentPosition() > armThreshold) {
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

                if (Arm.getCurrentPosition() > armThreshold && isLowered) {
                    if (upToggle) {
                        TurnTable.setTargetPosition(turnTableFront);
                        TurnTable.setPower(turnPower);
                    }
                    if (leftToggle) {
                        TurnTable.setTargetPosition(turnTableLeft);
                        TurnTable.setPower(turnPower);
                    }
                    if (rightToggle) {
                        TurnTable.setTargetPosition(turnTableRight);
                        TurnTable.setPower(turnPower);
                    }
                    if (downToggle) {
                        TurnTable.setTargetPosition(turnTableBack);
                        TurnTable.setPower(turnPower);
                    }
                    isLowered = false;

                    upToggle = false;
                    leftToggle = false;
                    rightToggle = false;
                    downToggle = false;
                } else if (-gamepad2.left_stick_y <= -.025) {
                    isLowered = false;

                    upToggle = false;
                    leftToggle = false;
                    rightToggle = false;
                    downToggle = false;
                }


                // Claw Controls
                if (gamepad2.b) {
                    Claw.setPosition(clawOpenPosition);
                    clawToggle = false;
                }

                if (gamepad2.a) {
                    Claw.setPosition(clawClosePosition);
                    clawToggle = true;
                }

                if(clawToggle){
                    Claw.setPosition(clawClosePosition);
                } else {
                    Claw.setPosition(clawOpenPosition);
                }


                // Extra TurnTable Macros
                if (gamepad2.x) {
                    leftMacro = true;
                }

                if (gamepad2.y) {
                    rightMacro = true;
                }


                if (leftMacro) {
                    LeftMacro();
                }

                if (rightMacro) {
                    RightMacro();
                }

            }


            {
                telemetry.addLine("Gamepad 1");
                telemetry.addData("Left Stick Y", -gamepad1.left_stick_y);
                telemetry.addData("Left Stick X", gamepad1.left_stick_x);
                telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
                telemetry.addData("Right Stick X", gamepad1.right_stick_x);

                telemetry.addLine();
                telemetry.addLine("Gamepad 2");
                telemetry.addData("Left Stick Y", -gamepad2.left_stick_y);
                telemetry.addData("Left Stick X", gamepad2.left_stick_x);
                telemetry.addData("Right Stick Y", gamepad2.right_stick_y);
                telemetry.addData("Right Stick X", gamepad2.right_stick_x);

                telemetry.addLine();
                telemetry.addLine("Motors");
                telemetry.addData("Front Left", FrontLeft.getCurrentPosition());
                telemetry.addData("Back Right", BackRight.getCurrentPosition());

                telemetry.addLine();
                telemetry.addLine("Turn Table");
                telemetry.addData("Turn Table Set Position", TurnTable.getTargetPosition());
                String turnTablePos = "IN MOVEMENT, ";
                if (Threshold.innerThreshold(TurnTable.getCurrentPosition(), turnTableFront, -50, 50)) {
                    turnTablePos = "FRONT, ";
                } else if (Threshold.innerThreshold(TurnTable.getCurrentPosition(), turnTableLeft, -50, 50)) {
                    turnTablePos = "LEFT, ";
                } else if (Threshold.innerThreshold(TurnTable.getCurrentPosition(), turnTableRight, -50, 50)) {
                    turnTablePos = "RIGHT, ";
                } else if (Threshold.innerThreshold(TurnTable.getCurrentPosition(), turnTableBack, -50, 50)) {
                    turnTablePos = "BACK, ";
                }
                telemetry.addData("Turn Table Position", turnTablePos + TurnTable.getCurrentPosition());

                telemetry.addLine();
                telemetry.addLine("Arm");
                telemetry.addData("Arm Set Position", Arm.getTargetPosition());
                telemetry.addData("Arm Position", Arm.getCurrentPosition());
                telemetry.addData("Arm Power", Arm.getPower());
                telemetry.addData("Arm Velocity", Arm.getVelocity());

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
            } // Telemetry Data
        }
    }

    // Mecanum Drive
    private void Mecanum(double Left_Y, double Left_X, double Right_X, float speedLimiter, double robotRoll, double robotPitch){
        FrontLeft.setPower(((Left_Y + Left_X + Right_X) * (Left_Y + Left_X + Right_X) * (Left_Y + Left_X + Right_X) / speedLimiter) + robotPitch - robotRoll);
        FrontRight.setPower(((Left_Y - Left_X - Right_X) *  (Left_Y - Left_X - Right_X) *  (Left_Y - Left_X - Right_X) / speedLimiter) + robotPitch + robotRoll);
        BackLeft.setPower(((Left_Y - Left_X + Right_X) * (Left_Y - Left_X + Right_X) * (Left_Y - Left_X + Right_X) / speedLimiter) + robotPitch + robotRoll);
        BackRight.setPower(((Left_Y + Left_X - Right_X) * (Left_Y + Left_X - Right_X) * (Left_Y + Left_X - Right_X) / speedLimiter) + robotPitch - robotRoll);
    }

    private void LeftMacro(){
        if(Threshold.innerThreshold(TurnTable.getCurrentPosition(), turnTableLeft, -50, 50)){
            Arm.setTargetPosition(0);
            Claw.setPosition(clawOpenPosition);
            leftMacro = false;
        } else if(Arm.getCurrentPosition() < armThreshold){
            Arm.setTargetPosition(armThreshold);
        } else if (Arm.getCurrentPosition() > armThreshold){
            TurnTable.setTargetPosition(turnTableLeft);
        }
    }

    private void RightMacro(){
        if(Threshold.innerThreshold(TurnTable.getCurrentPosition(), turnTableRight, -50, 50)){
            Arm.setTargetPosition(0);
            Claw.setPosition(clawOpenPosition);
            rightMacro = false;
        } else if(Arm.getCurrentPosition() < armThreshold){
            Arm.setTargetPosition(armThreshold);
        } else if (Arm.getCurrentPosition() > armThreshold){
            TurnTable.setTargetPosition(turnTableRight);
        }
    }
}