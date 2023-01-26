package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.other.PIDController;

@TeleOp(name = "Control Theory Testing")

public class ControlTheoryTesting extends LinearOpMode {

    private DcMotor TurnTable;


    public PIDController turnTableController = new PIDController();


    @Override
    public void runOpMode() {

        TurnTable = hardwareMap.get(DcMotor.class, "TurnTable");

        TurnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turnTableController.setPIDCoefficents(2, 0, 0);

        while(opModeIsActive()){
            TurnTable.setTargetPosition(100);
            turnTableController.updatePIDController(TurnTable.getTargetPosition(), TurnTable.getCurrentPosition());



            TurnTable.setPower(turnTableController.getOutput());
        }
    }
}
