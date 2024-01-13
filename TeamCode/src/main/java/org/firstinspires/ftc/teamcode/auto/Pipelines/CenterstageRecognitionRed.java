package org.firstinspires.ftc.teamcode.auto.Pipelines;

import static org.firstinspires.ftc.teamcode.auto.Pipelines.CenterstageRecognitionRed.CenterstagePosition.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class CenterstageRecognitionRed {
    public static boolean isRed = false;
    public enum CenterstagePosition{
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN,
    }

    private static volatile CenterstagePosition centerstagePosition = CENTER;

    static OpenCvWebcam leftWebcam;
    static OpenCvWebcam rightWebcam;
    static LeftWebcamPipelineRed leftWebcamPipelineRed;
    static RightWebcamPipelineRed rightWebcamPipelineRed;

    static LeftWebcamPipelineRed.LeftWebcamPosition leftWebcamPosition = LeftWebcamPipelineRed.LeftWebcamPosition.LEFT;
    static RightWebcamPipelineRed.RightWebcamPosition rightWebcamPosition = RightWebcamPipelineRed.RightWebcamPosition.RIGHT;
    static double rightWebcamDifferance = 0;
    static double leftWebcamDifferance = 0;

    static Telemetry telemetry;

    public static void init(HardwareMap hardwareMap, Telemetry tele){
        telemetry = tele;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        leftWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        rightWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

        leftWebcamPipelineRed = new LeftWebcamPipelineRed();
        rightWebcamPipelineRed = new RightWebcamPipelineRed();

        leftWebcam.setPipeline(leftWebcamPipelineRed);
        rightWebcam.setPipeline(rightWebcamPipelineRed);

        leftWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                leftWebcam.setPipeline(leftWebcamPipelineRed);
                leftWebcam.startStreaming(1280,960, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {}
        });

        rightWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                rightWebcam.setPipeline(rightWebcamPipelineRed);
                rightWebcam.startStreaming(1280,960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public static CenterstagePosition getPosition(){
        rightWebcamPosition = rightWebcamPipelineRed.getLastResult();
        leftWebcamPosition = leftWebcamPipelineRed.getLastResult();

        rightWebcamDifferance = rightWebcamPipelineRed.getRightCamDifferance();
        leftWebcamDifferance = leftWebcamPipelineRed.getLeftCamDifferance();

        if(rightWebcamPosition == RightWebcamPipelineRed.RightWebcamPosition.CENTER && leftWebcamPosition == LeftWebcamPipelineRed.LeftWebcamPosition.CENTER){
            centerstagePosition = CENTER;
        }else if(rightWebcamPosition == RightWebcamPipelineRed.RightWebcamPosition.RIGHT && rightWebcamDifferance > leftWebcamDifferance){
            centerstagePosition = RIGHT;
        }else if(leftWebcamPosition == LeftWebcamPipelineRed.LeftWebcamPosition.LEFT && leftWebcamDifferance > rightWebcamDifferance){
            centerstagePosition = LEFT;
        }else{
            centerstagePosition = UNKNOWN;
        }

        telemetry.addData("Right Cam", rightWebcamPipelineRed.getLastResult());
        telemetry.addData("Left Cam", leftWebcamPipelineRed.getLastResult());

        telemetry.addData("Right Cam", rightWebcamPipelineRed.getRightCamDifferance());
        telemetry.addData("Left Cam", leftWebcamPipelineRed.getLeftCamDifferance());

        telemetry.addData("Position", centerstagePosition);

        telemetry.update();

        return centerstagePosition;
    }
}
