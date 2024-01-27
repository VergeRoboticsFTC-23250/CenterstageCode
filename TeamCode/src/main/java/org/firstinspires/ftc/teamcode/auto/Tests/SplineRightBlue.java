package org.firstinspires.ftc.teamcode.auto.Tests;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class SplineRightBlue extends LinearOpMode {
    public static double a1 = 90;

    public static double delay1 = 1;
    public static double delay2 = 3.5;

    public static double x1 = 28;
    public static double y1 = 5;

    public static double x2 = 28;
    public static double y2 = -4.5;

    public static double x3 = 32;
    public static double y3 = 39;

    public static double x4 = 50;
    public static double y4 = 39;

    public static double x5 = 50;
    public static double y5 = -68;

    public static double x6 = 43;
    public static double y6 = -68;

    public void runOpMode(){
        Robot.init(hardwareMap);
        Robot.Claw.setRest();
        Robot.Arm.setRest();

        waitForStart();

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(new Pose2d())
                //Preload
                .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(a1)))
//                .addTemporalMarker(delay1, () -> {
//                    Robot.Arm.setIntake();
//                    Robot.Claw.setIntake();
//                })
                .lineTo(new Vector2d(x2, y2))
                //.addDisplacementMarker(() -> Robot.Claw.setRightGrip(true))
//                .addTemporalMarker(delay2, () -> {
//                    Robot.Claw.setBothGrips(false);
//                    Robot.Claw.setOuttake();
//                    Robot.Arm.setOuttake();
//                })
                .lineTo(new Vector2d(x3, y3))
                //.addDisplacementMarker(() -> Robot.Claw.setLeftGrip(true))
                .waitSeconds(0.5)
                //
                .lineTo(new Vector2d(x4, y4))
                .lineTo(new Vector2d(x5, y5))
                .waitSeconds(1)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(x6, y6))
                .resetConstraints()
                .build();
        drive.followTrajectorySequence(trajSeq1);
    }
}
