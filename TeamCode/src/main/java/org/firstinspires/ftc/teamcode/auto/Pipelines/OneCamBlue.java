package org.firstinspires.ftc.teamcode.auto.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import static org.firstinspires.ftc.teamcode.auto.Pipelines.OneCamBlue.PropPosition.*;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OneCamBlue extends OpenCvPipeline {
    public static Scalar targetColor = new Scalar(110, 255, 255);
    public Scalar rectColor = new Scalar(0, 0, 255);
    public Scalar recognizedColor = new Scalar(0, 255, 0);

    public static int height = 200;
    public static int width = 200;

    public static int leftRectX = 100;
    public static int leftRectY = 100;

    public static int centerRectX = 400;
    public static int centerRectY = 400;

    public static int rightRectX = 700;
    public static int rightRectY = 700;

    private Mat HSV = new Mat();
    private Mat output = new Mat();
    private Rect leftRect, centerRect, rightRect;
    private Mat leftCrop, centerCrop, rightCrop;
    private Scalar leftAvg, centerAvg, rightAvg;

    private double leftDist, centerDist, rightDist;

    public enum PropPosition{
        LEFT,
        CENTER,
        RIGHT
    }
    private volatile PropPosition position = CENTER;
    public void init(Mat firstFrame){
        leftRect = new Rect(leftRectX, leftRectY, width, height);
        centerRect = new Rect(centerRectX, centerRectY, width, height);
        rightRect = new Rect(rightRectX, rightRectY, width, height);
    }
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV_FULL);

        leftCrop = HSV.submat(leftRect);
        centerCrop = HSV.submat(centerRect);
        rightCrop = HSV.submat(rightRect);

        leftAvg = Core.mean(leftCrop);
        centerAvg = Core.mean(centerCrop);
        rightAvg = Core.mean(rightCrop);

        leftDist = calcDistance(leftAvg.val, targetColor.val);

        return output;
    }

    public PropPosition getLastPosition(){
        return position;
    }

    public double calcDistance(double[] avg, double[] target){
        double H = Math.min(Math.abs(avg[0] - target[0]), 180 - Math.abs(avg[0] - target[0])) / 180;
        double S = Math.abs(avg[1] - target[1]) / 255;
        double V = Math.abs(avg[2] - target[2]) / 255;

        return (H + S + V / 3)*100;
    }
}