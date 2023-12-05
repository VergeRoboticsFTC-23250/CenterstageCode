package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class Auto extends LinearOpMode {
    OpenCvWebcam webcam = null;
    CenterstagePipeline.CenterstagePosition resultingPosition = CenterstagePipeline.CenterstagePosition.LEFT;

    public void runOpMode(){
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        CenterstagePipeline pipeline = new CenterstagePipeline();
        webcam.setPipeline(pipeline);

        Robot.init(hardwareMap);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getLastResult());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        resultingPosition = pipeline.getLastResult();

        telemetry.addLine("Recognized " + resultingPosition);

        switch (resultingPosition)
        {
            case LEFT:
            {
                Robot.Slides.run(0, 1);
                break;
            }

            case RIGHT:
            {
                Robot.Slides.run(100, 1);
                break;
            }

            case CENTER:
            {
                Robot.Slides.run(200, 1);
                break;
            }
        }
    }
}
