package org.firstinspires.ftc.teamcode.auto.Tests;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class LineVsSpline extends LinearOpMode {
    public static double delay1 = 1;
    public static double delay2 = 3.5;
    public static double a1 = 90;
    public static double x1 = 28;
    public static double y1 = 5;
    public static double x2 = 28;
    public static double y2 = -4.5;

    public static double x3 = 32;
    public static double y3 = 39;
    public static double back1 = 6;
    public static double x4 = 50;
    public static double y4 = 0;
    public static double x6 = 43;
    public static double y6 = -68;

    public void runOpMode(){
        Robot.init(hardwareMap);
        Robot.Claw.setRest();
        Robot.Arm.setRest();

        waitForStart();

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(a1)))
                .addTemporalMarker(delay1, () -> {
                    Robot.Arm.setIntake();
                    Robot.Claw.setIntake();
                })
                .lineTo(new Vector2d(x2, y2))
                .addDisplacementMarker(() -> Robot.Claw.setRightGrip(true))
                .addTemporalMarker(delay2, () -> {
                    Robot.Claw.setBothGrips(false);
                    Robot.Claw.setOuttake();
                    Robot.Arm.setOuttake();
                })
                .lineTo(new Vector2d(x3, y3))
                .addDisplacementMarker(() -> {
                    Robot.Claw.setLeftGrip(true);
                    Robot.Arm.setRest();
                    Robot.Claw.setRest();
                })
                .lineTo(new Vector2d(x3, y3-back1))
                .lineTo(new Vector2d(x4, y4))
                .lineTo(new Vector2d(x4, y4-32))
                .lineTo(new Vector2d(x6, y6))
                .build();

        drive.followTrajectorySequence(trajSeq1);
    }
}
