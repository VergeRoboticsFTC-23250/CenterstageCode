package org.firstinspires.ftc.teamcode.auto;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class CenterstagePipeline extends OpenCvPipeline {
    Mat YCrCb = new Mat();
    Mat output = new Mat();
    Scalar rectColor = new Scalar(0, 0, 255);
    Scalar recognizedColor = new Scalar(0, 255, 0);

    Rect leftRect, centerRect, rightRect;

    Mat leftCrop, centerCrop, rightCrop;

    int avgLeft, avgCenter, avgRight;

    public enum CenterstagePosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    private volatile CenterstagePosition lastResult = CenterstagePosition.LEFT;

    public void init(Mat firstFrame){
        leftRect = new Rect(0, 0, firstFrame.cols() / 3, firstFrame.rows());
        centerRect = new Rect(firstFrame.cols() / 3, 0, firstFrame.cols() / 3, firstFrame.rows());
        rightRect = new Rect(firstFrame.cols() / 3 * 2, 0, firstFrame.cols() / 3, firstFrame.rows());
    }

    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        leftCrop = YCrCb.submat(leftRect);
        centerCrop = YCrCb.submat(centerRect);
        rightCrop = YCrCb.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop, 2);
        Core.extractChannel(centerCrop, centerCrop, 2);
        Core.extractChannel(rightCrop, rightCrop, 2);

        avgLeft = (int) Core.mean(leftCrop).val[0];
        avgCenter = (int) Core.mean(centerCrop).val[0];
        avgRight = (int) Core.mean(rightCrop).val[0];

        int max = Math.max(avgLeft, Math.max(avgCenter, avgRight));

        input.copyTo(output);
        Imgproc.rectangle(output, leftRect, rectColor, 2);
        Imgproc.rectangle(output, centerRect, rectColor, 2);
        Imgproc.rectangle(output, rightRect, rectColor, 2);

        if(max == avgLeft){
            lastResult = CenterstagePosition.LEFT;
            Imgproc.rectangle(output, leftRect, recognizedColor, 2);
        }else if(max == avgCenter){
            lastResult = CenterstagePosition.CENTER;
            Imgproc.rectangle(output, centerRect, recognizedColor, 2);
        } else{
            lastResult = CenterstagePosition.RIGHT;
            Imgproc.rectangle(output, rightRect, recognizedColor, 2);
        }

        return output;
    }

    public CenterstagePosition getLastResult(){
        return lastResult;
    }
}