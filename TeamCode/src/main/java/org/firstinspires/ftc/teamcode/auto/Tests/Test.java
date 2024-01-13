package org.firstinspires.ftc.teamcode.auto.Tests;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class Test extends LinearOpMode{
    public static double startHeading = 0;
    public static double x1 = 36;
    public static double y1 = 8;
    public static double a1 = 84;

    public static double x2 = 30;
    public static double y2 = 36;

    public static double x3 = 36;
    public static double y3 = 0;

    public static boolean b1 = false;
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        waitForStart();
        Robot.Claw.setLeftGrip(false);
        Robot.Claw.setRightGrip(false);

        Pose2d startPose = new Pose2d(0, 0, startHeading);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .setReversed(b1)
                .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(a1)))
                .addDisplacementMarker(() -> {
                    try{
                        Robot.Claw.setRightGrip(true);
                        Thread.sleep(200);
                        Robot.Claw.setRest();
                        Thread.sleep(200);
                        Robot.Arm.setOuttake();
                        Thread.sleep(200);
                        Robot.Claw.setOuttake();
                    }catch (InterruptedException e){

                    }
                })
                .lineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(a1)))
                .build();

        drive.followTrajectorySequence(trajSeq);

        Thread.sleep(500);
        Robot.Claw.setLeftGrip(true);
        Thread.sleep(500);
        Robot.Arm.setRest();


        Thread.sleep(2000);
    }
}