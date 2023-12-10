package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.util.Gamepads.Button.*;
import static org.firstinspires.ftc.teamcode.util.Robot.Outtake.PivotState.*;
import static org.firstinspires.ftc.teamcode.util.Robot.RobotState.*;

import org.firstinspires.ftc.teamcode.util.Gamepads;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp
public class DriverMode extends LinearOpMode {
    DriveThread driveThread = new DriveThread();
    TelemetryThread telemetryThread = new TelemetryThread();
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        Gamepads.update(gamepad1, gamepad2);

        waitForStart();

        driveThread.start();
        //telemetryThread.start();

        while (!isStopRequested()) {
            if(gamepad2.right_trigger > 0 && gamepad2.left_trigger > 0){
                Robot.Slides.run(0);
            }else if(gamepad2.right_trigger > 0){
                Robot.Slides.run(gamepad2.right_trigger);
            }else if(gamepad2.left_trigger > 0){
                Robot.Slides.run(-gamepad2.left_trigger);
            }else{
                Robot.Slides.run(0);
            }

            if(gamepad2.left_bumper || gamepad2.cross){
                Robot.Outtake.openGrip();
            }else{
                Robot.Outtake.closeGrip();
            }

            if(gamepad2.right_bumper){
                Robot.Outtake.setPivotState(UP);
            }else{
                Robot.Outtake.setPivotState(DOWN);
            }

            if(gamepad2.cross){
                Robot.Outtake.closeBlocker();
                Robot.Intake.setIsActive(true, gamepad2.square? 1 : gamepad2.triangle? -1 : Robot.Intake.speed);
                Robot.Outtake.openGrip();
            }else{
                Robot.Outtake.openBlocker();
                Robot.Intake.setIsActive(false, gamepad2.square? 1 : gamepad2.triangle? -1 : Robot.Intake.speed);
            }

            if(gamepad1.dpad_up && gamepad1.triangle){
                Robot.Airplane.launchPlane();
            }

            if(gamepad1.dpad_left && gamepad1.square){
                Robot.Hook.toggle(1);
                Thread.sleep(500);
            }
        }
    }

    class DriveThread extends Thread {
        public void run(){
            try{
                while (!isStopRequested()){
                    if(gamepad1.right_bumper){
                        Robot.Chassis.run(-gamepad1.right_stick_y * Robot.SLOW_AMOUNT, -gamepad1.right_stick_x * Robot.SLOW_AMOUNT, -gamepad1.left_stick_x * Robot.SLOW_AMOUNT);
                    }else{
                        Robot.Chassis.run(-gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
                    }
                }
            }catch (Exception e){

            }
        }
    }

    class TelemetryThread extends Thread {
        public void run(){
            try {
                while (!isStopRequested()){
                    telemetry.addData("Robot State", Robot.robotState);
                    telemetry.addData("Pivot State", Robot.Outtake.getPivotState());
                    telemetry.addData("Break Beam", Robot.BreakBeam.isBlocked());
                    telemetry.addData("Slide Position", Robot.Slides.getPos());
                    telemetry.addData("Slides isBusy", Robot.Slides.isBusy());
                    telemetry.addData("Slide Position Differance", Robot.Slides.getPosDifferance());
                    telemetry.addData("Slide Position individual", Robot.Slides.getPosIndividual());
                    telemetry.addData("Slide Target Pos individual", Robot.Slides.getTargetPositions());
                    telemetry.update();
                }

            }catch (Exception e){

            }
        }
    }
}