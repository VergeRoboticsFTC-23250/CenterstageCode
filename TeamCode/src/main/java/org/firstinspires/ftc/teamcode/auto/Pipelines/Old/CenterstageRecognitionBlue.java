package org.firstinspires.ftc.teamcode.auto.Pipelines.Old;

import static org.firstinspires.ftc.teamcode.auto.Pipelines.Old.CenterstageRecognitionBlue.CenterstagePosition.CENTER;
import static org.firstinspires.ftc.teamcode.auto.Pipelines.Old.CenterstageRecognitionBlue.CenterstagePosition.LEFT;
import static org.firstinspires.ftc.teamcode.auto.Pipelines.Old.CenterstageRecognitionBlue.CenterstagePosition.RIGHT;
import static org.firstinspires.ftc.teamcode.auto.Pipelines.Old.CenterstageRecognitionBlue.CenterstagePosition.UNKNOWN;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class CenterstageRecognitionBlue {
    public enum CenterstagePosition{
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN,
    }

    private static volatile CenterstagePosition centerstagePosition = CENTER;

    static OpenCvWebcam leftWebcam;
    static OpenCvWebcam rightWebcam;
    static LeftWebcamPipelineBlue leftWebcamPipelineBlue;
    static RightWebcamPipelineBlue rightWebcamPipelineBlue;

    static LeftWebcamPipelineBlue.LeftWebcamPosition leftWebcamPosition = LeftWebcamPipelineBlue.LeftWebcamPosition.LEFT;
    static RightWebcamPipelineBlue.RightWebcamPosition rightWebcamPosition = RightWebcamPipelineBlue.RightWebcamPosition.RIGHT;
    static double rightWebcamDifferance = 0;
    static double leftWebcamDifferance = 0;

    static Telemetry telemetry;

    public static void init(HardwareMap hardwareMap, Telemetry tele){
        telemetry = tele;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        leftWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        rightWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

        leftWebcamPipelineBlue = new LeftWebcamPipelineBlue();
        rightWebcamPipelineBlue = new RightWebcamPipelineBlue();

        leftWebcam.setPipeline(leftWebcamPipelineBlue);
        rightWebcam.setPipeline(rightWebcamPipelineBlue);

        leftWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                leftWebcam.setPipeline(leftWebcamPipelineBlue);
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
                rightWebcam.setPipeline(rightWebcamPipelineBlue);
                rightWebcam.startStreaming(1280,960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public static CenterstagePosition getPosition(){
        rightWebcamPosition = rightWebcamPipelineBlue.getLastResult();
        leftWebcamPosition = leftWebcamPipelineBlue.getLastResult();

        rightWebcamDifferance = rightWebcamPipelineBlue.getRightCamDifferance();
        leftWebcamDifferance = leftWebcamPipelineBlue.getLeftCamDifferance();

        if(rightWebcamPosition == RightWebcamPipelineBlue.RightWebcamPosition.CENTER && leftWebcamPosition == LeftWebcamPipelineBlue.LeftWebcamPosition.CENTER){
            centerstagePosition = CENTER;
        }else if(rightWebcamPosition == RightWebcamPipelineBlue.RightWebcamPosition.RIGHT && rightWebcamDifferance > leftWebcamDifferance){
            centerstagePosition = RIGHT;
        }else if(leftWebcamPosition == LeftWebcamPipelineBlue.LeftWebcamPosition.LEFT && leftWebcamDifferance > rightWebcamDifferance){
            centerstagePosition = LEFT;
        }else{
            centerstagePosition = UNKNOWN;
        }

        telemetry.addData("Right Cam", rightWebcamPipelineBlue.getLastResult());
        telemetry.addData("Left Cam", leftWebcamPipelineBlue.getLastResult());

        telemetry.addData("Right Cam", rightWebcamPipelineBlue.getRightCamDifferance());
        telemetry.addData("Left Cam", leftWebcamPipelineBlue.getLeftCamDifferance());

        telemetry.addData("Position", centerstagePosition);

        telemetry.update();

        return centerstagePosition;
    }
}
