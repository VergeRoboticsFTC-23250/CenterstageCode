package org.firstinspires.ftc.teamcode.auto.Tests;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
public class Test extends LinearOpMode{
    public static double power = 0.2;
    public static double power2 = 0.1;
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;
    public static double dist = -1;

    public static double dist2 = 3;

    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        Robot.Claw.setRest();
        Robot.Nicker.setOut();

        distanceSensor = hardwareMap.get(DistanceSensor.class, "color");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        colorSensor.enableLed(false);

        waitForStart();

        drive.setWeightedDrivePower(new Pose2d(-power, 0, 0));

        Thread.sleep(1000);

        drive.setWeightedDrivePower(new Pose2d());

        Pose2d lastPose = drive.getPoseEstimate();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(lastPose.getX() - dist, lastPose.getY(), 0))
                .build();

        drive.followTrajectorySequence(trajSeq2);

        Thread.sleep(500);

        drive.setWeightedDrivePower(new Pose2d(0, power2, 0));

        double myDist = distanceSensor.getDistance(DistanceUnit.CM);

        while (myDist > dist2){
            myDist = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("dist", myDist);
            telemetry.update();
        }

        drive.setWeightedDrivePower(new Pose2d());


        Robot.Nicker.setHome();

        Thread.sleep(2000);
    }
}