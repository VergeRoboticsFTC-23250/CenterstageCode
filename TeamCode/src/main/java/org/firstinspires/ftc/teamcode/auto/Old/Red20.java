package org.firstinspires.ftc.teamcode.auto.Old;
import static org.firstinspires.ftc.teamcode.auto.Pipelines.Old.CenterstageRecognitionRed.CenterstagePosition.*;
import static org.firstinspires.ftc.teamcode.auto.Pipelines.Old.CenterstageRecognitionRed.CenterstagePosition;
import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.Pipelines.Old.CenterstageRecognitionRed;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class Red20 extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        CenterstageRecognitionRed.init(hardwareMap, telemetry);
        CenterstagePosition position = UNKNOWN;
        Robot.init(hardwareMap);

        while (!isStarted() && !isStopRequested()){
            position = CenterstageRecognitionRed.getPosition();
            sleep(50);
        }

        waitForStart();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        if(position == LEFT){
            LeftPreload();
        }else if (position == RIGHT){
            RightPreload();
        }else {
            CenterPreload();
        }
    }

    void LeftPreload() throws InterruptedException {
        double back1 = 28;
        double turn1 = 86;
        double back2 = 3;
        int delay1 = 500;
        double forwardNig = 4;

        TrajectorySequence ts = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .back(back1)
                .turn(Math.toRadians(turn1))
                .back(back2)
                .build();
        drive.followTrajectorySequence(ts);

        Robot.Claw.setLeftGrip(true);
        Thread.sleep(delay1);

        TrajectorySequence ts0 = drive.trajectorySequenceBuilder(ts.end())
                .forward(forwardNig)
                .build();
        drive.followTrajectorySequence(ts0);

        Thread.sleep(2000);
    }

    void RightPreload() throws InterruptedException {
        double back1 = 28;
        double turn1 = 86;
        double forward0 = 20;
        double right1 = 8;
        double forward1 = 14;
        double forwardNig = 4;
        int delay1 = 500;

        TrajectorySequence ts = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .back(back1)
                .turn(Math.toRadians(turn1))
                .forward(forward0)
                .build();
        drive.followTrajectorySequence(ts);

        Robot.Claw.setLeftGrip(true);

        Thread.sleep(delay1);

        TrajectorySequence ts0 = drive.trajectorySequenceBuilder(ts.end())
                .forward(forwardNig)
                .build();

        drive.followTrajectorySequence(ts0);

        Thread.sleep(2000);
    }

    void CenterPreload() throws InterruptedException {
        double back1 = 29;
        double forward1 = 8;
        int delay1 = 500;

        TrajectorySequence ts = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .back(back1)
                .build();
        drive.followTrajectorySequence(ts);

        Robot.Claw.setLeftGrip(true);

        Thread.sleep(delay1);

        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(ts.end())
                .forward(forward1)
                .build();
        drive.followTrajectorySequence(ts2);
    }
}
