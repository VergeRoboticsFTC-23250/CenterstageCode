package org.firstinspires.ftc.teamcode.auto.Tests;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        Robot.Claw.setRest();

        waitForStart();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(18, 18), 0)
                .build();

        drive.followTrajectorySequence(traj1);
        Thread.sleep(2000);
    }
}