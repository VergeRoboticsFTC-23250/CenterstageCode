package org.firstinspires.ftc.teamcode.auto.Tests;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class SplineRightBlue extends LinearOpMode {
    ColorSensor color;
    public static double dist2 = 0.05;
    public static double dist1 = 4;
    public static double a1 = 90;
    public static double delay1 = 0.85;
    public static double delay2 = 4;
    public static double x1 = 26;
    public static double y1 = 5;
    public static double x2 = 26;
    public static double y2 = -4.5;
    public static double x3 = 34;
    public static double y3 = 40;
    public static double x4 = 54;
    public static double y4 = 0;
    public static double x5 = 54;
    public static double y5 = -60;
    public static double x6 = 48;
    public static double y6 = -67.5;
    public static double powerY = 0.15;
    public static double powerX = 0.25;
    public static double threshhold = 300;
    public static int pushTime = 750;
    public static int pushTime2 = 250;
    public static double backPower = 0.15;

    public static double offsetX = 10;
    public static double offsetY = 3;

    public static double time1 = 0.25;
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        Robot.Claw.setRest();
        Robot.Arm.setRest();

        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        color = hardwareMap.get(ColorSensor.class, "color");

        waitForStart();

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(a1)))
                .addTemporalMarker(delay1, Robot.Claw::setIntake)
                .lineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(a1)))
                .addDisplacementMarker(() -> Robot.Claw.setRightGrip(true))
                .addTemporalMarker(delay2, () -> {
                    Robot.Claw.setBothGrips(false);
                    Robot.Claw.setOuttake();
                    Robot.Arm.setOuttake();
                })
                .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(a1)))
                .addDisplacementMarker(() -> Robot.Claw.setLeftGrip(true))
                .waitSeconds(0.5)
                .back(dist1)
                .addDisplacementMarker(() -> {
                    Robot.Arm.setIntake();
                    Robot.Claw.setBothGrips(false);
                    Robot.Claw.setRest();
                })
                .lineToLinearHeading(new Pose2d(x4, y4, Math.toRadians(a1)))
                .lineToLinearHeading(new Pose2d(x5, y5, Math.toRadians(a1)))
                .addDisplacementMarker(Robot.Nicker::setOut)
                .lineToLinearHeading(new Pose2d(x6, y6, Math.toRadians(a1)))
                .build();

        drive.followTrajectorySequence(trajSeq1);
        Robot.Claw.setIntake();
        Thread.sleep(750);
        Robot.Claw.setBothGrips(true);
        drive.setWeightedDrivePower(new Pose2d(0, powerY, 0.003));
        while (hsvValues[2] < threshhold){
            Color.RGBToHSV((int) (color.red() * SCALE_FACTOR),
                    (int) (color.green() * SCALE_FACTOR),
                    (int) (color.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("H", hsvValues[0]);
            telemetry.addData("S", hsvValues[1]);
            telemetry.addData("V", hsvValues[2]);
            telemetry.update();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d());

        drive.setWeightedDrivePower(new Pose2d(-powerX, 0));
        Thread.sleep(pushTime);
        drive.setWeightedDrivePower(new Pose2d(backPower, 0, 0));
        Thread.sleep(pushTime2);
        drive.setWeightedDrivePower(new Pose2d());

        Robot.Nicker.setHome();

        Thread.sleep(750);
        Robot.Claw.setBothGrips(false);

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(x5, y5, Math.toRadians(a1)))
                        .addTemporalMarker(time1, () -> {
                            Robot.Nicker.setOut();
                        })
                        .addTemporalMarker(time1+.125, () -> {
                            Robot.Nicker.setRest();
                        })
                        .addTemporalMarker(time1+.5, () -> {
                            Robot.Nicker.setOut();
                        })
                        .addTemporalMarker(time1 + 1, () -> {
                            Robot.Arm.setRest();
                            Robot.Claw.setRest();
                        })
                        .addTemporalMarker(time1 + 2, () -> {
                            Robot.Claw.setBothGrips(true);
                        })
                        .addTemporalMarker(time1 + 2.5, () -> {
                            Robot.Claw.setBothGrips(false);
                        })
                        .lineToLinearHeading(new Pose2d(x4, y4, Math.toRadians(a1)))
                        .addDisplacementMarker(() -> {
                            Robot.Claw.setOuttake();
                            Robot.Arm.setOuttake();
                        })
                .lineToLinearHeading(new Pose2d(x3 - offsetX, y3 + offsetY, Math.toRadians(a1)))
                .build()
        );

        Robot.Claw.setBothGrips(true);

        Thread.sleep(500);

        Robot.Arm.setRest();

        Thread.sleep(2000);
    }
}
