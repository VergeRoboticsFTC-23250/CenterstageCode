package org.firstinspires.ftc.teamcode.auto.Tests;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class SplineLeft extends LinearOpMode {
    public static double x1 = 28;
    public static double y1 = 18;
    public static double a1 = 90;

    public static double x2 = 32;
    public static double y2 = 39;
    public static double forward1 = 8;
    public static double x3 = 52;
    public static double y3 = 0;
    public static double x4 = 46;
    public static double y4 = -68;

    public static double delay1 = 1;
    public static double back1 = 22;
    public static double power = 0.2;
    public static double power2 = -0.15;
    public static double dist = 0.75;

    public static double back2 = 48;
    public static double offset1 = 8;
    public static double offset2 = 0;
    public static double dist2 = 2.5;
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;
    DigitalChannel limit;

    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        Robot.Claw.setRest();

        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color");
        limit = hardwareMap.get(DigitalChannel.class, "limit");

        waitForStart();

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
                .addDisplacementMarker(() -> {
                    Robot.Claw.setRightGrip(false);
                    Robot.Claw.setLeftGrip(true);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Robot.Arm.setRest();
                    Robot.Claw.setRest();
                })
                .lineToSplineHeading(new Pose2d(x3, y3, Math.toRadians(a1)))
                .back(back2)
                .addDisplacementMarker(() -> {
                    Robot.Claw.setBothGrips(false);
                    Robot.Nicker.setOut();
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Robot.Arm.setIntake();
                    Robot.Claw.setIntake();
                })
                .lineToSplineHeading(new Pose2d(x4, y4, Math.toRadians(a1)))
                .build();

        drive.followTrajectorySequence(trajSeq1);

        drive.setWeightedDrivePower(new Pose2d(-power, 0, 0));

        Robot.Claw.setBothGrips(true);

        Thread.sleep(1000);

        drive.setWeightedDrivePower(new Pose2d());

        Pose2d lastPose = drive.getPoseEstimate();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(lastPose.getX(), lastPose.getY() + dist, Math.toRadians(a1)))
                .build();

        drive.followTrajectorySequence(trajSeq2);

        drive.setWeightedDrivePower(new Pose2d(0, power2, 0));

        double myDist = distanceSensor.getDistance(DistanceUnit.CM);

        while (myDist > dist2){
            myDist = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("dist", myDist);
            telemetry.update();
        }

        drive.setWeightedDrivePower(new Pose2d());

        Robot.Nicker.setRest();

        Thread.sleep(1000);

        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(x4, y4, Math.toRadians(a1)))
                .lineToSplineHeading(new Pose2d(x3, y3 - back2, Math.toRadians(a1)))
                .addDisplacementMarker(() -> Robot.Claw.setBothGrips(false))
                .lineToSplineHeading(new Pose2d(x3, y3, Math.toRadians(a1)))
                .addDisplacementMarker(() -> {
                    Robot.Nicker.setOut();
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Robot.Claw.setOuttake();
                    Robot.Arm.setOuttake();
                })
                .lineToSplineHeading(new Pose2d(x2-offset1, y2+offset2, Math.toRadians(a1)))
                .build();

        drive.followTrajectorySequence(trajSeq3);

        Robot.Claw.setBothGrips(true);

        Thread.sleep(2000);
    }
}
