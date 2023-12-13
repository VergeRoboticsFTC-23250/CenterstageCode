package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.Pipelines.LeftWebcamPipeline;
import org.firstinspires.ftc.teamcode.auto.Pipelines.RightWebcamPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class Auto extends LinearOpMode {
    OpenCvWebcam leftWebcam;
    OpenCvWebcam rightWebcam;
    LeftWebcamPipeline leftWebcamPipeline;
    RightWebcamPipeline rightWebcamPipeline;

    LeftWebcamPipeline.LeftWebcamPosition leftWebcamPosition = LeftWebcamPipeline.LeftWebcamPosition.LEFT;
    RightWebcamPipeline.RightWebcamPosition rightWebcamPosition = RightWebcamPipeline.RightWebcamPosition.RIGHT;
    double rightWebcamDifferance = 0;
    double leftWebcamDifferance = 0;
    public void runOpMode() throws InterruptedException {
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

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Right Cam", rightWebcamPipeline.getLastResult());
            telemetry.addData("Left Cam", leftWebcamPipeline.getLastResult());

            rightWebcamPosition = rightWebcamPipeline.getLastResult();
            leftWebcamPosition = leftWebcamPipeline.getLastResult();

            telemetry.addData("Right Cam", rightWebcamPipeline.getRightCamDifferance());
            telemetry.addData("Left Cam", leftWebcamPipeline.getLeftCamDifferance());

            rightWebcamDifferance = rightWebcamPipeline.getRightCamDifferance();
            leftWebcamDifferance = leftWebcamPipeline.getLeftCamDifferance();

            if(rightWebcamPosition == RightWebcamPipeline.RightWebcamPosition.CENTER && leftWebcamPosition == LeftWebcamPipeline.LeftWebcamPosition.CENTER){
                telemetry.addData("Position", "CENTER");
            }else if(rightWebcamPosition == RightWebcamPipeline.RightWebcamPosition.RIGHT && rightWebcamDifferance > leftWebcamDifferance){
                telemetry.addData("Position", "RIGHT");
            }else if(leftWebcamPosition == LeftWebcamPipeline.LeftWebcamPosition.LEFT && leftWebcamDifferance > rightWebcamDifferance){
                telemetry.addData("Position", "LEFT");
            }else{
                telemetry.addData("Position", "UNKNOWN");
            }

            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Left cam FPS", leftWebcam.getFps());
            telemetry.addData("Right cam FPS", rightWebcam.getFps());
            telemetry.update();

            sleep(100);
        }
    }
}
