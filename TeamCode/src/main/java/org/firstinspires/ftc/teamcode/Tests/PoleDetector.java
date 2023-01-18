package org.firstinspires.ftc.teamcode.Tests;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PoleDetector extends OpenCvPipeline{
    Telemetry telemetry;
    Mat mat = new Mat();

    public int loca;

    public enum Location {
        FL,
        FR,
        CL,
        CR,
        M,
        NOT_FOUND
    }

    public Location location;

    static final Rect Pole_ROI_FL = new Rect( // Correct
            new Point(0, 0),
            new Point(427, 720));
    static final Rect Pole_ROI_FR = new Rect(
            new Point(851, 0),
            new Point(1280, 720));
    static final Rect Pole_ROI_CL = new Rect(
            new Point(427, 0),
            new Point(586, 720));
    static final Rect Pole_ROI_CR = new Rect(
            new Point(692, 0),
            new Point(851, 720));
    static final Rect Pole_ROI_M = new Rect(
            new Point(586, 0),
            new Point(692, 720));

    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public PoleDetector(Telemetry t) {telemetry = t;}

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // HSV Ranges       Still need values
        Scalar lowHSV = new Scalar(32, 69, 133);
        Scalar highHSV = new Scalar(48, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat pLocaFL = mat.submat(Pole_ROI_FL);
        Mat pLocaFR = mat.submat(Pole_ROI_FR);
        Mat pLocaCL = mat.submat(Pole_ROI_CL);
        Mat pLocaCR = mat.submat(Pole_ROI_CR);
        Mat pLocaM = mat.submat(Pole_ROI_M);

        double FLValue = Core.sumElems(pLocaFL).val[0] / Pole_ROI_FL.area() / 255;
        double FRValue = Core.sumElems(pLocaFR).val[0] / Pole_ROI_FR.area() / 255;
        double CLValue = Core.sumElems(pLocaCL).val[0] / Pole_ROI_CL.area() / 255;
        double CRValue = Core.sumElems(pLocaCR).val[0] / Pole_ROI_CR.area() / 255;
        double MValue = Core.sumElems(pLocaM).val[0] / Pole_ROI_M.area() / 255;

        pLocaFL.release();
        pLocaFR.release();
        pLocaCL.release();
        pLocaCR.release();
        pLocaM.release();

        telemetry.addData("Far Left raw value", (int) Core.sumElems(pLocaFL).val[0]);
        telemetry.addData("Far Right raw value", (int) Core.sumElems(pLocaFL).val[0]);
        telemetry.addData("Close Left raw value", (int) Core.sumElems(pLocaFL).val[0]);
        telemetry.addData("Close Right raw value", (int) Core.sumElems(pLocaFL).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(pLocaFL).val[0]);
        telemetry.addData("Far Right percentage", Math.round(FLValue * 100) + "%");
        telemetry.addData("Close Left percentage", Math.round(FLValue * 100) + "%");
        telemetry.addData("Close Right percentage", Math.round(FLValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(FLValue * 100) + "%");

        boolean fLeft = FLValue > PERCENT_COLOR_THRESHOLD;
        boolean fRight = FRValue > PERCENT_COLOR_THRESHOLD;
        boolean cLeft = CLValue > PERCENT_COLOR_THRESHOLD;
        boolean cRight = CRValue > PERCENT_COLOR_THRESHOLD;
        boolean middle = MValue > PERCENT_COLOR_THRESHOLD;


        if (fLeft && fRight && cLeft && cRight && middle || fLeft && fRight && cLeft && cRight || fLeft && fRight && cLeft && cRight || fLeft && fRight && cLeft || fLeft && fRight){
            location = PoleDetector.Location.NOT_FOUND;
        } else if (fLeft){
            location = PoleDetector.Location.FL;
        } else if (fRight){
            location = PoleDetector.Location.FR;
        } else if (cLeft){
            location = PoleDetector.Location.CL;
        } else if (cRight){
            location = PoleDetector.Location.CR;
        } else if (middle){
            location = PoleDetector.Location.M;
        }

        Scalar JunctionColorL = new Scalar(41, 14, 149);
        Scalar JunctionColorH = new Scalar(78, 55, 186);

        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(mat, Pole_ROI_FL, location == Location.FL? JunctionColorL:JunctionColorH);
        Imgproc.rectangle(mat, Pole_ROI_FR, location == Location.FR? JunctionColorL:JunctionColorH);
        Imgproc.rectangle(mat, Pole_ROI_CL, location == Location.CL? JunctionColorL:JunctionColorH);
        Imgproc.rectangle(mat, Pole_ROI_CR, location == Location.CR? JunctionColorL:JunctionColorH);
        Imgproc.rectangle(mat, Pole_ROI_M, location == Location.M? JunctionColorL:JunctionColorH);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}