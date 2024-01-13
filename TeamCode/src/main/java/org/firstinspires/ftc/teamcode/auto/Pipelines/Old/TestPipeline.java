package org.firstinspires.ftc.teamcode.auto.Pipelines.Old;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestPipeline extends OpenCvPipeline {
    Mat color = new Mat();
    Mat output = new Mat();
    Scalar rectColor = new Scalar(0, 0, 255);
    Scalar recognizedColor = new Scalar(0, 255, 0);

    Rect leftRect, centerRect;

    Mat leftCrop, rightCrop;

    double avgLeft, avgRight;

    private volatile double left = 0;
    private volatile double right = 0;

    public void init(Mat firstFrame){
        leftRect = new Rect(0, 0, firstFrame.cols() / 3, firstFrame.rows());
        centerRect = new Rect(firstFrame.cols() / 3, 0, firstFrame.cols() / 3, firstFrame.rows());
    }

    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, color, Imgproc.COLOR_RGB2HSV);

        leftCrop = color.submat(leftRect);
        rightCrop = color.submat(centerRect);

        Core.extractChannel(leftCrop, leftCrop, 0);
        Core.extractChannel(rightCrop, rightCrop, 0);

        avgLeft = Core.mean(leftCrop).val[0];
        avgRight = Core.mean(rightCrop).val[0];

        left = avgLeft;
        right = avgRight;

        input.copyTo(output);
        Imgproc.rectangle(output, leftRect, rectColor, 2);
        Imgproc.rectangle(output, centerRect, rectColor, 2);

        return output;
    }

    public String getLeftCamDifferance(){
        return (left + " | " + right);
    }
}
