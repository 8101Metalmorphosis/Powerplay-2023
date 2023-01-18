package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


class SleeveDetector extends OpenCvPipeline
{

    Telemetry telemetry;

    Mat matR = new Mat();
    Mat matG = new Mat();
    Mat matB = new Mat();

    public enum Location {
        P1,
        P2,
        P3,
        NOT_FOUND
    }


    private Location location;

    static final Rect Sleeve_ROI = new Rect(
            new Point(30, 25),
            new Point(90, 100));


    public SleeveDetector(Telemetry t) {telemetry = t;}

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, matR, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, matG, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, matB, Imgproc.COLOR_RGB2HSV);


        Scalar lowRED = new Scalar(160, 50, 50);
        Scalar highRED = new Scalar(180, 255, 255);

        Scalar lowGREEN = new Scalar(35, 50, 50);
        Scalar highGREEN = new Scalar(55, 255, 255);

        Scalar lowBLUE = new Scalar(95, 50, 50);
        Scalar highBLUE = new Scalar(120, 255, 255);

        Core.inRange(matR, lowRED, highRED, matR);
        Core.inRange(matG, lowGREEN, highGREEN, matG);
        Core.inRange(matB, lowBLUE, highBLUE, matB);

        Mat red = matR.submat(Sleeve_ROI);
        Mat green = matG.submat(Sleeve_ROI);
        Mat blue = matB.submat(Sleeve_ROI);

        double rValue = Core.sumElems(red).val[0] / Sleeve_ROI.area() / 255;
        double gValue = Core.sumElems(green).val[0] / Sleeve_ROI.area() / 255;
        double bValue = Core.sumElems(blue).val[0] / Sleeve_ROI.area() / 255;


        telemetry.addData("Red raw value", (int) Core.sumElems(red).val[0]);
        telemetry.addData("Green raw value", (int) Core.sumElems(green).val[0]);
        telemetry.addData("Blue raw value", (int) Core.sumElems(blue).val[0]);

        telemetry.addData("Red percentage", Math.round(rValue * 100) + "%");
        telemetry.addData("Green percentage", Math.round(gValue * 100) + "%");
        telemetry.addData("Blue percentage", Math.round(bValue * 100) + "%");


        red.release();
        green.release();
        blue.release();

        double cRed = rValue;
        double cGreen = gValue;
        double cBlue = bValue;


        if (cBlue > cGreen && cBlue > cRed) {
            location = Location.P3;
            telemetry.addData("Parking Location", "3");
        } else if (cGreen > cRed && cGreen > cBlue){
            location = Location.P2;
            telemetry.addData("Parking Location", "2");
        } else if (Math.round(gValue * 100) < 8 && Math.round(bValue * 100) < 8){
            location = Location.P1;
            telemetry.addData("Parking Location", "1");
        } else {
            location = Location.NOT_FOUND;
            telemetry.addData("Parking Location", "Not Found");
        }
        telemetry.update();


        Imgproc.rectangle(matR, Sleeve_ROI, location == Location.P1? lowRED:highRED);
        Imgproc.rectangle(matG, Sleeve_ROI, location == Location.P2? lowGREEN:highGREEN);
        Imgproc.rectangle(matB, Sleeve_ROI, location == Location.P3? lowBLUE:highBLUE);

        return matR;
    }

    public Location getLocation(){
        return location;
    }
}