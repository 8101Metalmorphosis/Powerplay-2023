package org.firstinspires.ftc.teamcode.other;

public class MotionProfile {

    private double maxAccel;
    private double maxDccel;
    private double setDistance;
    private double currentDistance;

    private double offset;
    private double halfwayDistance;

    private double estimatedTD;

    private double velocity;

    public void setValues(double maxAccel, double maxDccel, double maxVelocity){
        this.maxAccel = maxAccel;
        this.maxDccel = maxDccel;
    }

    public void setPositions(double setDistance, double currentDistance){
        this.currentDistance = currentDistance;
        this.setDistance = setDistance;
        offset = setDistance - currentDistance;
    }

    public void updateProfile(){
        halfwayDistance = offset / 2;


        if(offset < halfwayDistance){
            velocity = velocity + maxAccel;
        } else {
            velocity = velocity + maxDccel;
        }
    }

    public void resetProfile(){
        velocity = 0;
    }

    public double getVelocity(){
        return velocity;
    }
}
