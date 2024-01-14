package org.firstinspires.ftc.teamcode.auto.Pipelines;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.*;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition;

public class YCrCbBlue extends OpenCvPipeline {
    Mat YCrCb = new Mat();
    Mat output = new Mat();
    Scalar rectColor = new Scalar(0, 0, 255);
    Scalar recognizedColor = new Scalar(0, 255, 0);

    Rect leftRect, centerRect, rightRect;

    Mat leftCrop, centerCrop, rightCrop;

    double avgLeft, avgRight, avgCenter;

    public static int height = 200;
    public static int width = 200;
    public static double centerSizeOffset = 1;

    public static int leftRectX = 100;
    public static int leftRectY = 100;

    public static int centerRectX = 400;
    public static int centerRectY = 400;

    public static int rightRectX = 700;
    public static int rightRectY = 700;
    private volatile PropPosition position = CENTER;

    public void init(Mat firstFrame){
        leftRect = new Rect(leftRectX, leftRectY, width, height);
        centerRect = new Rect(centerRectX, centerRectY, (int)(width * centerSizeOffset), (int)(height * centerSizeOffset));
        rightRect = new Rect(rightRectX, rightRectY, width, height);
    }

    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        leftCrop = YCrCb.submat(leftRect);
        centerCrop = YCrCb.submat(centerRect);
        rightCrop = YCrCb.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop, 2);
        Core.extractChannel(centerCrop, centerCrop, 2);
        Core.extractChannel(rightCrop, rightCrop, 2);

        avgLeft = Core.mean(leftCrop).val[0];
        avgCenter = Core.mean(centerCrop).val[0];
        avgRight = Core.mean(rightCrop).val[0];

        input.copyTo(output);
        Imgproc.rectangle(output, leftRect, rectColor, 4);
        Imgproc.rectangle(output, centerRect, rectColor, 4);
        Imgproc.rectangle(output, rightRect, rectColor, 4);

        double max = Math.max(avgCenter, Math.max(avgRight, avgLeft));

        if(max == avgLeft){
            position = LEFT;
            Imgproc.rectangle(output, leftRect, recognizedColor, 4);
        }else if(max == avgRight){
            position = RIGHT;
            Imgproc.rectangle(output, rightRect, recognizedColor, 4);
        }else{
            position = CENTER;
            Imgproc.rectangle(output, leftRect, recognizedColor, 4);
        }

        return output;
    }

    public PropPosition getLastPosition(){
        return position;
    }

    public String getAverages(){
        return " Left: " + avgLeft + "\n Center: " + avgCenter + "\n Right: " + avgRight;
    }
}
