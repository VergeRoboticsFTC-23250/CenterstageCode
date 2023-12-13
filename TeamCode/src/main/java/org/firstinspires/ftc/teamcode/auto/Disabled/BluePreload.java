package org.firstinspires.ftc.teamcode.auto.Disabled;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
public class BluePreload extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        waitForStart();
        Robot.Outtake.openBlocker();
        Robot.Outtake.closeGrip();
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(26)
                .build();
        drive.followTrajectory(traj1);

        Robot.Slides.run(1450, 1);

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(24)
                .build();
        drive.followTrajectory(traj2);

        Robot.Outtake.setPivotState(Robot.Outtake.PivotState.UP);
        Thread.sleep(2000);
        Robot.Outtake.openGrip();
        Thread.sleep(500);
        Robot.Outtake.setPivotState(Robot.Outtake.PivotState.DOWN);
        Thread.sleep(1000);
        Robot.Slides.run(0, 1);

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(23)
                .build();
        drive.followTrajectory(traj3);

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(8)
                .build();
        drive.followTrajectory(traj4);
    }
}