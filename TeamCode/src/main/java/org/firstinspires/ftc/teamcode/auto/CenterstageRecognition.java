package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.auto.CenterstageRecognition.CenterstagePosition.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.Pipelines.LeftWebcamPipeline;
import org.firstinspires.ftc.teamcode.auto.Pipelines.RightWebcamPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class CenterstageRecognition {
    public enum CenterstagePosition{
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN,
    }

    public static volatile CenterstagePosition centerstagePosition = CENTER;

    static OpenCvWebcam leftWebcam;
    static OpenCvWebcam rightWebcam;
    static LeftWebcamPipeline leftWebcamPipeline;
    static RightWebcamPipeline rightWebcamPipeline;

    static LeftWebcamPipeline.LeftWebcamPosition leftWebcamPosition = LeftWebcamPipeline.LeftWebcamPosition.LEFT;
    static RightWebcamPipeline.RightWebcamPosition rightWebcamPosition = RightWebcamPipeline.RightWebcamPosition.RIGHT;
    static double rightWebcamDifferance = 0;
    static double leftWebcamDifferance = 0;

    public static void init(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        leftWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        rightWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

        leftWebcamPipeline = new LeftWebcamPipeline();
        rightWebcamPipeline = new RightWebcamPipeline();

        leftWebcam.setPipeline(leftWebcamPipeline);
        rightWebcam.setPipeline(rightWebcamPipeline);

        leftWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                leftWebcam.setPipeline(leftWebcamPipeline);
                leftWebcam.startStreaming(1280,960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        rightWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                rightWebcam.setPipeline(rightWebcamPipeline);
                rightWebcam.startStreaming(1280,960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public static void update(){
        rightWebcamPosition = rightWebcamPipeline.getLastResult();
        leftWebcamPosition = leftWebcamPipeline.getLastResult();

        telemetry.addData("Right Cam", rightWebcamPipeline.getLastResult());
        telemetry.addData("Left Cam", leftWebcamPipeline.getLastResult());

        rightWebcamDifferance = rightWebcamPipeline.getRightCamDifferance();
        leftWebcamDifferance = leftWebcamPipeline.getLeftCamDifferance();

        telemetry.addData("Right Cam", rightWebcamPipeline.getRightCamDifferance());
        telemetry.addData("Left Cam", leftWebcamPipeline.getLeftCamDifferance());

        if(rightWebcamPosition == RightWebcamPipeline.RightWebcamPosition.CENTER && leftWebcamPosition == LeftWebcamPipeline.LeftWebcamPosition.CENTER){
            centerstagePosition = CENTER;
            telemetry.addData("Position", "CENTER");
        }else if(rightWebcamPosition == RightWebcamPipeline.RightWebcamPosition.RIGHT && rightWebcamDifferance > leftWebcamDifferance){
            centerstagePosition = RIGHT;
            telemetry.addData("Position", "RIGHT");
        }else if(leftWebcamPosition == LeftWebcamPipeline.LeftWebcamPosition.LEFT && leftWebcamDifferance > rightWebcamDifferance){
            centerstagePosition = LEFT;
            telemetry.addData("Position", "LEFT");
        }else{
            centerstagePosition = UNKNOWN;
            telemetry.addData("Position", "UNKNOWN");
        }

        telemetry.update();
    }
}
