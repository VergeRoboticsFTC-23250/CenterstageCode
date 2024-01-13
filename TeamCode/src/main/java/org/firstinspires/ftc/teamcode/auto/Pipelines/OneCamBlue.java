package org.firstinspires.ftc.teamcode.auto.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OneCamBlue extends OpenCvPipeline {
    Mat YCrCb = new Mat();
    Mat output = new Mat();
    Scalar rectColor = new Scalar(0, 0, 255);
    Scalar recognizedColor = new Scalar(0, 255, 0);

    Rect centerRect, rightRect;

    Mat centerCrop, rightCrop;

    double avgCenter, avgRight;

    public enum RightWebcamPosition
    {
        RIGHT,
        CENTER,
    }

    private volatile RightWebcamPosition lastResult = RightWebcamPosition.RIGHT;
    private volatile double leftCamDifferance = 0;

    public void init(Mat firstFrame){
        centerRect = new Rect(0, 0, firstFrame.cols() / 3 * 2, firstFrame.rows() / 3);
        rightRect = new Rect(firstFrame.cols() / 3 * 2, 0, firstFrame.cols() / 3, firstFrame.rows() / 3 * 2);
    }

    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        centerCrop = YCrCb.submat(centerRect);
        rightCrop = YCrCb.submat(rightRect);

        Core.extractChannel(centerCrop, centerCrop, 2);
        Core.extractChannel(rightCrop, rightCrop, 2);

        avgCenter = Core.mean(centerCrop).val[0];
        avgRight = Core.mean(rightCrop).val[0];

        leftCamDifferance = Math.abs(avgCenter - avgRight);

        input.copyTo(output);
        Imgproc.rectangle(output, centerRect, rectColor, 2);
        Imgproc.rectangle(output, rightRect, rectColor, 2);

        if(avgCenter > avgRight){
            lastResult = RightWebcamPosition.CENTER;
            Imgproc.rectangle(output, centerRect, recognizedColor, 2);
        }else{
            lastResult = RightWebcamPosition.RIGHT;
            Imgproc.rectangle(output, rightRect, recognizedColor, 2);
        }

        return output;
    }

    public RightWebcamPosition getLastResult(){
        return lastResult;
    }

    public double getRightCamDifferance(){
        return leftCamDifferance;
    }
}
