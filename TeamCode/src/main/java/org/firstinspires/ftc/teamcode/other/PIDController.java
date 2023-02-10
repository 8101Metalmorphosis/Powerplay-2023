package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double Kp;
    private double Ki;
    private double Kd;

    private double error;
    private double lastError;

    private double position;

    private double derivative;
    private double integralSum;

    public double output;


    ElapsedTime timer = new ElapsedTime();

    public void setPIDCoefficents(double p, double i, double d) {
        Kp = p;
        Ki = i;
        Kd = d;
    }

    public void updatePIDController(double reference, double currentPosition) {
        position = currentPosition;

        error = reference - position;

        derivative = (error - lastError) / timer.seconds();

        integralSum = integralSum + (error * timer.seconds());


        output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;
    }

    public void resetTimer(){
        timer = new ElapsedTime();
    }

    public double getOutput(){
        return output;
    }

    public double getKp() {
        return Kp;
    }

    public double getKi() {
        return Ki;
    }

    public double getKd() {
        return Kd;
    }

}

