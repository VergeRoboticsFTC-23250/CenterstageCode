package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.Pipelines.DetectionBlue;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.Robot.*;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.*;

@Autonomous
@Config
public class Blue70 extends LinearOpMode {
    public static double centerPark = 24;
    public static double leftPark = 20;
    public static double rightPark = 32;
    PropPosition position = CENTER;
    public void runOpMode() throws InterruptedException {
        DetectionBlue.init(hardwareMap, telemetry);
        Robot.init(hardwareMap);
        Robot.Claw.setRest();

        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Detected", DetectionBlue.getPosition());
            telemetry.addData("Averages", DetectionBlue.pipeline.getAverages());
            telemetry.update();
            position = DetectionBlue.getPosition();
            sleep(50);
        }

        waitForStart();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        if(position == LEFT){
            LeftPreload();
        }else if (position == RIGHT){
            RightPreload();
        }else{
            CenterPreload();
        }
    }

    private void CenterPreload() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, 0);

        double x1 = 36;
        double y1 = 8;
        double a1 = 90;

        double x2 = 27;
        double y2 = 39;
        double forward1 = 8;

        double delay1 = 1;

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(delay1, Robot.Claw::setIntake)
                .lineToSplineHeading(new Pose2d(x1, y1, Math.toRadians(a1)))
                .addDisplacementMarker(() -> Robot.Claw.setRightGrip(true))
                .forward(forward1)
                .addDisplacementMarker(() -> {
                    Robot.Claw.setRightGrip(false);
                    Robot.Claw.setOuttake();
                    Robot.Arm.setOuttake();
                })
                .lineToSplineHeading(new Pose2d(x2, y2, Math.toRadians(a1)))
                .build();

        drive.followTrajectorySequence(trajSeq1);

        Robot.Claw.setRightGrip(false);
        Robot.Claw.setLeftGrip(true);
        Thread.sleep(500);
        Robot.Arm.setRest();
        Robot.Claw.setRest();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .back(2)
                .strafeLeft(centerPark)
                .build();

        drive.followTrajectorySequence(trajSeq2);
    }

    private void RightPreload() throws InterruptedException {
        double x1 = 28;
        double y1 = 18;
        double a1 = 90;
        double x2 = 34;
        double y2 = 38;
        double forward1 = 8;

        double delay1 = 1;
        double back1 = 22;

        Pose2d startPose = new Pose2d(0, 0, 0);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(delay1, Robot.Claw::setIntake)
                .lineToSplineHeading(new Pose2d(x1, y1, Math.toRadians(a1)))
                .back(back1)
                .addDisplacementMarker(() -> Robot.Claw.setRightGrip(true))
                .forward(forward1)
                .addDisplacementMarker(() -> {
                    Robot.Claw.setRightGrip(false);
                    Robot.Claw.setOuttake();
                    Robot.Arm.setOuttake();
                })
                .lineToSplineHeading(new Pose2d(x2, y2, Math.toRadians(a1)))
                .build();

        drive.followTrajectorySequence(trajSeq1);

        Robot.Claw.setRightGrip(false);
        Robot.Claw.setLeftGrip(true);
        Thread.sleep(500);
        Robot.Arm.setRest();
        Robot.Claw.setRest();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .back(2)
                .strafeLeft(rightPark)
                .build();

        drive.followTrajectorySequence(trajSeq2);
    }

    private void LeftPreload() throws InterruptedException {
        double x1 = 28;
        double y1 = 18;
        double a1 = 90;

        double x2 = 20;
        double y2 = 36;
        double forward1 = 8;

        double delay1 = 1;

        Pose2d startPose = new Pose2d(0, 0, 0);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(delay1, Robot.Claw::setIntake)
                .lineToSplineHeading(new Pose2d(x1, y1, Math.toRadians(a1)))
                .addDisplacementMarker(() -> Robot.Claw.setRightGrip(true))
                .forward(forward1)
                .addDisplacementMarker(() -> {
                    Robot.Claw.setRightGrip(false);
                    Robot.Claw.setOuttake();
                    Robot.Arm.setOuttake();
                })
                .lineToSplineHeading(new Pose2d(x2, y2, Math.toRadians(a1)))
                .build();

        drive.followTrajectorySequence(trajSeq1);

        Robot.Claw.setRightGrip(false);
        Robot.Claw.setLeftGrip(true);
        Thread.sleep(500);
        Robot.Arm.setRest();
        Robot.Claw.setRest();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .back(2)
                .strafeLeft(leftPark)
                .build();

        drive.followTrajectorySequence(trajSeq2);
    }

}
