package org.firstinspires.ftc.teamcode.auto.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.*;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OneCamBlue extends OpenCvPipeline {
    public static Scalar targetColor = new Scalar(110, 255, 255);
    public static Scalar wrongColor = new Scalar(255, 0, 255);
    public static Scalar correctColor = new Scalar(0, 255, 0);

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
    private int leftDist, centerDist, rightDist;
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
        centerDist = calcDistance(centerAvg.val, targetColor.val);
        rightDist = calcDistance(rightAvg.val, targetColor.val);

        int max = Math.max(leftDist, Math.max(centerDist, rightDist));

        position = max == leftDist? LEFT : max == rightDist ? RIGHT : CENTER;

        input.copyTo(output);

        Imgproc.rectangle(output, leftRect, wrongColor, 4);
        Imgproc.rectangle(output, centerRect, wrongColor, 4);
        Imgproc.rectangle(output, rightRect, wrongColor, 4);

        Imgproc.rectangle(output, position == LEFT? leftRect : position == RIGHT? rightRect : centerRect, correctColor, 4);

        return output;
    }

    public PropPosition getLastPosition(){
        return position;
    }

    public int calcDistance(double[] avg, double[] target){
        double H = Math.min(Math.abs(avg[0] - target[0]), 180 - Math.abs(avg[0] - target[0])) / 180;
        double S = Math.abs(avg[1] - target[1]) / 255;
        double V = Math.abs(avg[2] - target[2]) / 255;

        return (int)((H + S + V / 3)*1000);
    }

    private String scalarToString(Scalar scalar){
        return "H: " + scalar.val[0] + " S: " + scalar.val[1] + " V: " + scalar.val[2];
    }

    public String getColors(){
        return "Left: " + scalarToString(leftAvg) + " Center: " + scalarToString(centerAvg) + " Right: " + scalarToString(rightAvg);
    }

    public String getDistances(){
        return "Left: " + leftDist + " Center: " + centerDist + " Right: " + rightDist;
    }
}