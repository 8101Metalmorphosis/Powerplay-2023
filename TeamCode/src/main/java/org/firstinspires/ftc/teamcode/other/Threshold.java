package org.firstinspires.ftc.teamcode.other;

public class Threshold {

    public static boolean innerThreshold(double value, double value2, double low, double high){
        boolean inThreshold = false;

        if(value < value2 + high && value > value2 + low){
            inThreshold = true;
        }

        return inThreshold;
    }

    public static boolean innerThresholdEqual(double value, double value2, double low, double high){
        boolean inThreshold = false;

        if(value <= value2 + high && value >= value2 - low){
            inThreshold = true;
        }

            return inThreshold;
    }

    public static boolean outerThreshold(double value, double value2, double low, double high){
        boolean inThreshold = false;

        if(value > value2 + high && value < value2 - low){
            inThreshold = true;
        }

        return inThreshold;
    }

    public static boolean outerThresholdEqual(double value, double value2, double low, double high){
        boolean inThreshold = false;

        if(value >= value2 + high && value <= value2 - low){
            inThreshold = true;
        }

        return inThreshold;
    }
}
