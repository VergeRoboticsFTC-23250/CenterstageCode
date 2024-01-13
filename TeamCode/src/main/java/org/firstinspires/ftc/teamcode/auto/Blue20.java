package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.Pipelines.CenterstageRecognitionBlue.CenterstagePosition.LEFT;
import static org.firstinspires.ftc.teamcode.auto.Pipelines.CenterstageRecognitionBlue.CenterstagePosition.RIGHT;
import static org.firstinspires.ftc.teamcode.auto.Pipelines.CenterstageRecognitionBlue.CenterstagePosition.UNKNOWN;
import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.Pipelines.CenterstageRecognitionBlue;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

@Autonomous
public class Blue20 extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        CenterstageRecognitionBlue.init(hardwareMap, telemetry);
        CenterstageRecognitionBlue.CenterstagePosition position = UNKNOWN;
        Robot.init(hardwareMap);

        while (!isStarted() && !isStopRequested()){
            position = CenterstageRecognitionBlue.getPosition();
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

    void LeftPreload() throws InterruptedException {
        double back1 = 28;
        double turn1 = -86;
        double forward0 = 19;
        double forwardNig = 4;
        int delay1 = 500;

        TrajectorySequence ts = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .back(back1)
                .turn(Math.toRadians(turn1))
                .forward(forward0)
                .build();
        drive.followTrajectorySequence(ts);

        Robot.Claw.setRightGrip(true);

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

        Robot.Claw.setRightGrip(true);

        Thread.sleep(delay1);

        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(ts.end())
                .forward(forward1)
                .build();
        drive.followTrajectorySequence(ts2);

        Thread.sleep(2000);
    }

    void RightPreload() throws InterruptedException {
        double back1 = 28;
        double turn1 = -86;
        double back2 = 3;
        int delay1 = 500;
        double forwardNig = 4;

        TrajectorySequence ts = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .back(back1)
                .turn(Math.toRadians(turn1))
                .back(back2)
                .build();
        drive.followTrajectorySequence(ts);

        Robot.Claw.setRightGrip(true);
        Thread.sleep(delay1);

        TrajectorySequence ts0 = drive.trajectorySequenceBuilder(ts.end())
                .forward(forwardNig)
                .build();
        drive.followTrajectorySequence(ts0);

        Thread.sleep(2000);
    }
}
