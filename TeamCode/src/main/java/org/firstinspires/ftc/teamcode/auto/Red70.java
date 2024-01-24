package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.Pipelines.DetectionBlue;
import org.firstinspires.ftc.teamcode.auto.Pipelines.DetectionRed;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.Robot.*;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.*;

@Autonomous
public class Red70 extends LinearOpMode {
    public static double centerPark = 24;
    public static double leftPark = 24;
    public static double rightPark = 30;
    PropPosition position = CENTER;
    public void runOpMode() throws InterruptedException {
        DetectionRed.init(hardwareMap, telemetry);
        Robot.init(hardwareMap);
        Robot.Claw.setRest();

        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Detected", DetectionRed.getPosition());
            telemetry.addData("Averages", DetectionRed.pipeline.getAverages());
            telemetry.update();
            sleep(50);
            position = DetectionRed.getPosition();
        }

        waitForStart();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        if(position == LEFT){
            RightPreload();
        }else if (position == RIGHT){
            LeftPreload();
        }else{
            CenterPreload();
        }

        Thread.sleep(2000);
    }

    private void CenterPreload() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, 0);

        double x1 = 36;
        double y1 = -8;
        double a1 = -90;

        double x2 = 27;
        double y2 = -37;
        double forward1 = 8;

        double delay1 = 1;

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(delay1, Robot.Claw::setIntake)
                .lineToSplineHeading(new Pose2d(x1, y1, Math.toRadians(a1)))
                .addDisplacementMarker(() -> Robot.Claw.setLeftGrip(true))
                .forward(forward1)
                .addDisplacementMarker(() -> {
                    Robot.Claw.setLeftGrip(false);
                    Robot.Claw.setOuttake();
                    Robot.Arm.setOuttake();
                })
                .lineToSplineHeading(new Pose2d(x2, y2, Math.toRadians(a1)))
                .build();

        drive.followTrajectorySequence(trajSeq1);

        Robot.Claw.setLeftGrip(false);
        Robot.Claw.setRightGrip(true);
        Thread.sleep(500);
        Robot.Arm.setRest();
        Robot.Claw.setRest();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .strafeRight(centerPark)
                .build();

        drive.followTrajectorySequence(trajSeq2);
    }

    private void RightPreload() throws InterruptedException {
        double x1 = 28;
        double y1 = -18;
        double a1 = -90;
        double x2 = 32;
        double y2 = -36;
        double forward1 = 8;

        double delay1 = 1;
        double back1 = 22;

        Pose2d startPose = new Pose2d(0, 0, 0);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(delay1, Robot.Claw::setIntake)
                .lineToSplineHeading(new Pose2d(x1, y1, Math.toRadians(a1)))
                .back(back1)
                .addDisplacementMarker(() -> Robot.Claw.setLeftGrip(true))
                .forward(forward1)
                .addDisplacementMarker(() -> {
                    Robot.Claw.setLeftGrip(false);
                    Robot.Claw.setOuttake();
                    Robot.Arm.setOuttake();
                })
                .lineToSplineHeading(new Pose2d(x2, y2, Math.toRadians(a1)))
                .build();

        drive.followTrajectorySequence(trajSeq1);

        Robot.Claw.setLeftGrip(false);
        Robot.Claw.setRightGrip(true);
        Thread.sleep(500);
        Robot.Arm.setRest();
        Robot.Claw.setRest();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .strafeRight(rightPark)
                .build();

        drive.followTrajectorySequence(trajSeq2);
    }

    private void LeftPreload() throws InterruptedException {
        double x1 = 28;
        double y1 = -18;
        double a1 = -90;

        double x2 = 18;
        double y2 = -35;
        double forward1 = 8;

        double delay1 = 1;

        Pose2d startPose = new Pose2d(0, 0, 0);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(delay1, Robot.Claw::setIntake)
                .lineToSplineHeading(new Pose2d(x1, y1, Math.toRadians(a1)))
                .addDisplacementMarker(() -> Robot.Claw.setLeftGrip(true))
                .forward(forward1)
                .addDisplacementMarker(() -> {
                    Robot.Claw.setLeftGrip(false);
                    Robot.Claw.setOuttake();
                    Robot.Arm.setOuttake();
                })
                .lineToSplineHeading(new Pose2d(x2, y2, Math.toRadians(a1)))
                .build();

        drive.followTrajectorySequence(trajSeq1);

        Robot.Claw.setLeftGrip(false);
        Robot.Claw.setRightGrip(true);
        Thread.sleep(500);
        Robot.Arm.setRest();
        Robot.Claw.setRest();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .strafeRight(leftPark)
                .build();

        drive.followTrajectorySequence(trajSeq2);
    }
}
