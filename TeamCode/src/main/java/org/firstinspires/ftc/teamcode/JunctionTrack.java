package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class JunctionTrack extends OpenCvPipeline {

    Mat mat = new Mat();

    private Telemetry telemetry;

    public static Rect rect;
    public static Rect targetedRect;


    static double[] widths = new double[10];

    static double maxWidth;
    static int maxWidthArr;


    double cameraCenter = 320 / 2;
    static double junctionOffset;


    public static int currentPosition = JunctionTeleOp.turnTablePos;
    public static int targetPosition = 0;



    public JunctionTrack(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowOrange = new Scalar(8, 90, 110);
        Scalar highOrange = new Scalar(14, 255, 255);

        Core.inRange(mat, lowOrange, highOrange, mat);


        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


        for (int i = 0; i < contours.size(); i++) {
            Imgproc.drawContours(input, contours, i, new Scalar(0, 255, 0), 1);

            rect = Imgproc.boundingRect(contours.get(i));


            widths[i] = rect.width;
        }

        maxWidth = findWidth();

        targetedRect = Imgproc.boundingRect(contours.get(maxWidthArr));

        Imgproc.rectangle(input, new Point(targetedRect.x, targetedRect.y),
                new Point(targetedRect.x + targetedRect.width, targetedRect.y + targetedRect.height),
                new Scalar(0, 0, 255), 2);


        junctionOffset = (targetedRect.x - cameraCenter) + (targetedRect.width / 2);

        Imgproc.putText(input, "Target", new Point(targetedRect.x, targetedRect.y - 5), 2, .5, new Scalar(255, 255, 255));
        Imgproc.circle(input, new Point(targetedRect.x + (targetedRect.width / 2), targetedRect.y + (targetedRect.height / 2)), 1, new Scalar(255, 255, 0), 4);


        telemetry.addData("Targeted X", targetedRect.x);
        telemetry.addData("Targeted Width", targetedRect.width);
        telemetry.addData("Set Contour", maxWidthArr);

        telemetry.addData("Camera Center", cameraCenter);
        telemetry.addData("Junction Offset", junctionOffset);

        targetPosition = calculatePosition();
        if(targetPosition <= -1400){
            targetPosition = 700;
        } else if (targetPosition >= 800){
            targetPosition = -1300;
        }


        telemetry.addData("TurnTable Position", currentPosition);
        telemetry.addData("TurnTable TargetPosition", targetPosition);


        for (int i = 0; i < widths.length; i++) {
            telemetry.addData("Widths", widths[i]);
        }

        telemetry.update();


        return input;
    }

    public static int calculatePosition(){
        int calculatedTarget = (int) Math.round(currentPosition + junctionOffset);

        return calculatedTarget;
    }

    public static double findWidth(){
        double max = widths[0];

        for (int i = 0; i < widths.length; i++) {
            if(widths[i] > max){
                max = widths[i];
                maxWidthArr = i;
            }
        }

        return max;
    }

}
