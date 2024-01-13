package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.Pipelines.DetectionBlue;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.Robot.*;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.*;

@Autonomous
public class Blue70 extends LinearOpMode {
    PropPosition position = CENTER;
    public void runOpMode() throws InterruptedException {
        DetectionBlue.init(hardwareMap, telemetry);
        Robot.init(hardwareMap);

        while (!isStarted() && !isStopRequested()){
            position = DetectionBlue.getPosition();
            sleep(50);
        }

        waitForStart();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        if(position == LEFT){
            LeftPreload();
        }else if (position == RIGHT){
            RightPreload();
        }else{
            CenterPreload();
        }
    }

    private void CenterPreload() {
    }

    private void RightPreload() {
    }

    private void LeftPreload() {
    }
}
