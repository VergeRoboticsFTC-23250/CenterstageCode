package org.firstinspires.ftc.teamcode.auto.Tests;

import static org.firstinspires.ftc.teamcode.auto.Tests.SplineRightBlue.backPower;
import static org.firstinspires.ftc.teamcode.auto.Tests.SplineRightBlue.dist2;
import static org.firstinspires.ftc.teamcode.auto.Tests.SplineRightBlue.powerX;
import static org.firstinspires.ftc.teamcode.auto.Tests.SplineRightBlue.powerY;
import static org.firstinspires.ftc.teamcode.auto.Tests.SplineRightBlue.pushTime;
import static org.firstinspires.ftc.teamcode.auto.Tests.SplineRightBlue.pushTime2;
import static org.firstinspires.ftc.teamcode.auto.Tests.SplineRightBlue.threshhold;
import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
@Config
public class Test extends LinearOpMode{
    ColorSensor color;
    public static int waitTime = 0;
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        Robot.Nicker.setOut();
        Robot.Arm.setIntake();
        Robot.Claw.setIntake();
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        color = hardwareMap.get(ColorSensor.class, "color");

        waitForStart();

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

        Thread.sleep(1500);
        Robot.Claw.setBothGrips(false);
        Thread.sleep(500);
        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(8)
                        .build()
        );
        Robot.Nicker.setOut();
        Thread.sleep(500);
        Robot.Claw.setRest();

        Thread.sleep(3000);
    }
}