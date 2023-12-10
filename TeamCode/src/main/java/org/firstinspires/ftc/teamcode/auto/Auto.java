package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.auto.CenterstagePipeline.CenterstagePosition.LEFT;
import static org.firstinspires.ftc.teamcode.auto.CenterstagePipeline.CenterstagePosition.RIGHT;
import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

@Autonomous
public class Auto extends LinearOpMode {
    OpenCvWebcam webcam = null;
    CenterstagePipeline.CenterstagePosition resultingPosition = LEFT;

    public void runOpMode() throws InterruptedException {
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

        if(resultingPosition != LEFT){
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(10)
                    .build()
            );
        }else{
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(6)
                    .build()
            );
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                    .forward(24)
                    .build()
            );
            drive.turn(Math.toRadians(90));
        }

        Thread.sleep(1000);

        resultingPosition = pipeline.getLastResult();

        if(resultingPosition == RIGHT){
            //RIGHT
        }else{
            //CENTER
        }

        telemetry.addData("Realtime analysis", resultingPosition);
        telemetry.update();
    }
}
