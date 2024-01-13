package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Robot;

import static org.firstinspires.ftc.teamcode.util.Robot.*;

@TeleOp
public class DriverMode extends LinearOpMode {
    DriveThread driveThread = new DriveThread();
    TelemetryThread telemetryThread = new TelemetryThread();
    ClawThread clawThread = new ClawThread();
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);

        waitForStart();

        IntakeToRest();

        driveThread.start();
        telemetryThread.start();
        clawThread.start();

        while (!isStopRequested()) {
            if(gamepad2.cross && !gamepad2.square){
                if(robotState != RobotState.INTAKE){
                    RestToIntake();
                }
            }else if(gamepad2.square && !gamepad2.cross){
                if(robotState != RobotState.OUTTAKE){
                    RestToOuttake();
                }

                if(gamepad2.right_trigger > 0 && gamepad2.left_trigger > 0){
                    Slides.run(0);
                }else if(gamepad2.right_trigger > 0){
                    Slides.run(gamepad2.right_trigger);
                }else if(gamepad2.left_trigger > 0){
                    Slides.run(-gamepad2.left_trigger);
                }else{
                    Slides.run(0);
                }

            }else{
                if(robotState == RobotState.INTAKE){
                    IntakeToRest();
                }else if(robotState == RobotState.OUTTAKE){
                    OuttakeToRest();
                }
            }

            if(gamepad1.dpad_up && gamepad1.triangle){
                Airplane.launchPlane();
            }

            if(gamepad1.dpad_left && gamepad1.square){
                while (gamepad1.dpad_left || gamepad1.square){}
                Hook.toggle(1);
            }
        }
    }

    class DriveThread extends Thread {
        public void run(){
            try{
                while (!isStopRequested()){
                    if(gamepad1.right_bumper){
                        Robot.Chassis.run(-gamepad1.right_stick_y * Robot.SLOW_SPEED, -gamepad1.right_stick_x * Robot.SLOW_SPEED, -gamepad1.left_stick_x * Robot.SLOW_SPEED);
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
                    telemetry.addData("Slide Position", Robot.Slides.getPos());
                    telemetry.addData("Slides isBusy", Robot.Slides.isBusy());
                    telemetry.addData("Slide Position Differance", Robot.Slides.getPosDifferance());
                    telemetry.addData("Slide Position individual", Robot.Slides.getPosIndividual());
                    telemetry.addData("Slide Target Pos individual", Robot.Slides.getTargetPositions());
                    telemetry.addData("Limits", LimitSwitch.getLeft() + " | " + LimitSwitch.getRight());
                    telemetry.addData("RobotState", robotState);
                    telemetry.update();
                }

            }catch (Exception e){

            }
        }
    }

    class ClawThread extends Thread {
        public void run(){
            try {
                while (!isStopRequested()){
                    Claw.setRightGrip(gamepad2.right_bumper);
                    Claw.setLeftGrip(gamepad2.left_bumper);

                    if(LimitSwitch.getRight() && LimitSwitch.getLeft()){
                        gamepad2.setLedColor(0, 1, 0, 1000);
                        gamepad1.setLedColor(0, 1, 0, 1000);
                        gamepad2.rumble(300);
                        gamepad1.rumble(300);
                        Thread.sleep(300);
                        gamepad1.rumble(300);
                        gamepad2.rumble(300);
                    }else if(LimitSwitch.getLeft() || LimitSwitch.getRight()){
                        gamepad2.setLedColor(1, 0, 1, 1000);
                        gamepad1.setLedColor(1, 0, 1, 1000);
                        gamepad1.rumble(300);
                        gamepad2.rumble(300);
                    }
                }
            }catch (Exception e){

            }
        }
    }
}