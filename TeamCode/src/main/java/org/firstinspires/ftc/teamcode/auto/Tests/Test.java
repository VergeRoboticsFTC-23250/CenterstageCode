package org.firstinspires.ftc.teamcode.auto.Tests;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class Test extends LinearOpMode{
    public static double startHeading = 0;
    public static double x1 = 36;
    public static double y1 = 8;
    public static double a1 = 85.75;

    public static double x2 = 29;
    public static double y2 = 34;

    public static double x3 = 64;
    public static double y3 = 30;

    public static double x4 = 48;
    public static double y4 = -48;
    public static double x5 = 38;
    public static double y5 = -60;
    public static double back1 = 10;

    public static boolean b1 = false;
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        waitForStart();

        Pose2d startPose = new Pose2d(0, 0, startHeading);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(b1)
                .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(a1)))
                .addDisplacementMarker(() -> {
                    try{
                        Robot.Claw.setRightGrip(true);
                        Thread.sleep(200);
                        Robot.Claw.setRest();
                        Thread.sleep(200);
                        //Robot.Arm.setOuttake();
                        Thread.sleep(200);
                        Robot.Claw.setOuttake();
                    }catch (InterruptedException e){

                    }
                })
                .lineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(a1)))
                .build();

        drive.followTrajectorySequence(trajSeq1);

        Robot.Claw.setLeftGrip(true);
        Thread.sleep(200);
        Robot.Claw.setLeftGrip(false);
        Robot.Claw.setRightGrip(false);
        //Robot.Arm.setRest();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(a1)))
                .lineToLinearHeading(new Pose2d(x4, y4, Math.toRadians(a1)))
                .lineToLinearHeading(new Pose2d(x5, y5, Math.toRadians(a1)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .addDisplacementMarker(() -> {
                    Robot.Claw.setBothGrips(false);
                    Robot.Nicker.setOut();
                    Robot.Claw.setIntake();
                })
                .back(back1)
                .build();

        drive.followTrajectorySequence(trajSeq2);
        Robot.Claw.setBothGrips(false);
        Thread.sleep(500);
        Robot.Nicker.setHome();

        Thread.sleep(2000);
    }
}