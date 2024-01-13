package org.firstinspires.ftc.teamcode.auto.Pipelines.Old;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class LeftWebcamPipelineBlue extends OpenCvPipeline {
    Mat YCrCb = new Mat();
    Mat output = new Mat();
    Scalar rectColor = new Scalar(0, 0, 255);
    Scalar recognizedColor = new Scalar(0, 255, 0);

    Rect leftRect, centerRect;

    Mat leftCrop, rightCrop;

    double avgLeft, avgRight;

    public enum LeftWebcamPosition
    {
        LEFT,
        CENTER,
    }

    private volatile LeftWebcamPosition lastResult = LeftWebcamPosition.LEFT;
    private volatile double leftCamDifferance = 0;

    public void init(Mat firstFrame){
        leftRect = new Rect(0, 0, firstFrame.cols() / 3, firstFrame.rows() / 3 * 2);
        centerRect = new Rect(firstFrame.cols() / 3, 0, firstFrame.cols() / 3 * 2, firstFrame.rows() / 3);
    }

    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        leftCrop = YCrCb.submat(leftRect);
        rightCrop = YCrCb.submat(centerRect);

        Core.extractChannel(leftCrop, leftCrop, 2);
        Core.extractChannel(rightCrop, rightCrop, 2);

        avgLeft = Core.mean(leftCrop).val[0];
        avgRight = Core.mean(rightCrop).val[0];

        leftCamDifferance = Math.abs(avgLeft - avgRight);

        input.copyTo(output);
        Imgproc.rectangle(output, leftRect, rectColor, 2);
        Imgproc.rectangle(output, centerRect, rectColor, 2);

        if(avgLeft > avgRight){
            lastResult = LeftWebcamPosition.LEFT;
            Imgproc.rectangle(output, leftRect, recognizedColor, 2);
        }else{
            lastResult = LeftWebcamPosition.CENTER;
            Imgproc.rectangle(output, centerRect, recognizedColor, 2);
        }

        return output;
    }

    public LeftWebcamPosition getLastResult(){
        return lastResult;
    }

    public double getLeftCamDifferance(){
        return leftCamDifferance;
    }
}
