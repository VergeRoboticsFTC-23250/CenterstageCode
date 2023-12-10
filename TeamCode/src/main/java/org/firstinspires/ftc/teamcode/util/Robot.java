package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.Robot.Outtake.PivotState.*;
import static org.firstinspires.ftc.teamcode.util.Robot.RobotState.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class Robot {
    public static double SLOW_AMOUNT = 0.3;
    public static enum RobotState{
        INTAKE,
        SCORING,
    }

    public static RobotState robotState = INTAKE;
    public static void init(HardwareMap hardwareMap){
        Chassis.init(hardwareMap);
        Slides.init(hardwareMap);
        Hook.init(hardwareMap);
        Intake.init(hardwareMap);
        Outtake.init(hardwareMap);
        BreakBeam.init(hardwareMap);
        Airplane.init(hardwareMap);
    }

    public static class Chassis{
        public static SampleMecanumDrive drive;

        public static void init(HardwareMap hardwareMap){
            drive = new SampleMecanumDrive(hardwareMap);
            Chassis.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public static Pose2d run(double x, double y, double heading){
            drive.setWeightedDrivePower(
                    new Pose2d(x, y, heading)
            );

            drive.update();
            return drive.getPoseEstimate();
        }
    }

    @Config
    public static class Slides{
        public static DcMotor leftSlideMotor;
        public static DcMotor rightSlideMotor;

        public static int MAX = 2750;
        public static int MIN = 0;

        public static void init(HardwareMap hardwareMap){
            leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlide");
            rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlide");

            rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public static int getPos(){
            return (int)Math.round((leftSlideMotor.getCurrentPosition() + rightSlideMotor.getCurrentPosition()) / 2.0);
        }

        public static int getPosDifferance(){
            return leftSlideMotor.getCurrentPosition() - rightSlideMotor.getCurrentPosition();
        }

        public static String getPosIndividual(){
            return "Left: " + leftSlideMotor.getCurrentPosition() + "Right: " + rightSlideMotor.getCurrentPosition();
        }

        public static boolean isBusy(){
            return leftSlideMotor.isBusy() && rightSlideMotor.isBusy();
        }

        public static void run(double pow){
            int pos = getPos();

            if(pos < MAX && pos > MIN || pos >= MAX && pow <= 0 || pos <= MIN && pow >= 0){
                leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                leftSlideMotor.setPower(pow);
                rightSlideMotor.setPower(pow);
            }else{
                leftSlideMotor.setPower(0);
                rightSlideMotor.setPower(0);
            }
        }

        public static void run(int pos, double pow){
            leftSlideMotor.setPower(0);
            rightSlideMotor.setPower(0);

            leftSlideMotor.setTargetPosition(pos);
            rightSlideMotor.setTargetPosition(pos);

            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlideMotor.setPower(pow);
            rightSlideMotor.setPower(pow);

            while (isBusy()){}

            leftSlideMotor.setPower(0);
            rightSlideMotor.setPower(0);
        }

        public static String getTargetPositions(){
            return "Left Side: " + leftSlideMotor.getTargetPosition() + " Right Side: " + rightSlideMotor.getTargetPosition();
        }

        public static void home(double pow){
            run(0, pow);
            try {
                Thread.sleep(500);
            }catch (Exception e){

            }
            leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    @Config
    public static class Hook{
        public static int maxPos = 5500;
        private static DcMotor hookMotor;

        private static boolean isExtended = false;

        public static void init(HardwareMap hardwareMap){
            hookMotor = hardwareMap.get(DcMotor.class, "hook");
            hookMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public static void toggle(int pow){
            hookMotor.setPower(0);
            isExtended = !isExtended;
            hookMotor.setTargetPosition(isExtended? maxPos : 1000);
            hookMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hookMotor.setPower(pow);
        }
    }

    @Config
    public static class Intake{
        private static DcMotor motor;
        public static Servo pivot;

        private static boolean isActive = false;
        public static double speed = 0.8;
        public static double upPosition = 0.5;
        public static double downPosition = 0.8;

        public static void init(HardwareMap hardwareMap){
            motor = hardwareMap.get(DcMotor.class, "intake");
            pivot = hardwareMap.get(Servo.class, "intakePivot");
            pivot.setPosition(upPosition);
        }

        public static void toggle(){
            isActive = !isActive;
            motor.setPower(isActive? speed : 0);
            pivot.setPosition(isActive? downPosition : upPosition);
        }

        public static void setIsActive(boolean isA, double spd){
            isActive = isA;
            motor.setPower(isActive? spd : 0);
            pivot.setPosition(isActive? downPosition : upPosition);
        }
    }

    @Config
    public static class Outtake{
        private static Servo pivot;
        private static Servo grip;
        private static Servo blocker;

        public static double PIVOT_UP = 1;
        public static double PIVOT_REST = 0.5;
        public static double PIVOT_DOWN = 0;
        public static double GRIP_CLOSE = 0.6;
        public static double BLOCK_BLOCK = 0;
        public static double BLOCK_OPEN = 0.8;

        public enum PivotState {
            UP,
            REST,
            DOWN
        }

        private static PivotState pivotState = DOWN;

        public static void init(HardwareMap hardwareMap){
            pivot = hardwareMap.get(Servo.class, "outtakePivot");
            Outtake.setPivotState(DOWN);

            grip = hardwareMap.get(Servo.class, "outtakeGrip");
            grip.setPosition(GRIP_CLOSE);

            blocker = hardwareMap.get(Servo.class, "blocker");
            blocker.setPosition(BLOCK_BLOCK);
        }

        public static void openBlocker(){
            blocker.setPosition(BLOCK_OPEN);
        }

        public static void closeBlocker(){
            blocker.setPosition(BLOCK_BLOCK);
        }

        public static PivotState getPivotState(){
            return pivotState;
        }

        public static void openGrip(){
            grip.setPosition(GRIP_CLOSE + 0.15);
        }

        public static void closeGrip(){
            grip.setPosition(GRIP_CLOSE);
        }

        public static double getPos(){
            return pivot.getPosition();
        }

        public static void setPivotState(PivotState state){
            pivotState = state;

            if(pivotState == UP){
                pivot.setPosition(PIVOT_UP);
            }else if(pivotState == REST){
                pivot.setPosition(PIVOT_REST);
            }else if(pivotState == DOWN){
                pivot.setPosition(PIVOT_DOWN);
            }
        }
    }

    public static class BreakBeam{
        static DigitalChannel breakBeam;
        public static void init(HardwareMap hardwareMap){
            breakBeam = hardwareMap.get(DigitalChannel.class, "breakBeam");
            breakBeam.setMode(DigitalChannel.Mode.INPUT);
        }

        public static boolean isBlocked(){
            return breakBeam.getState();
        }
    }

    @Config
    public static class Airplane{
        public static Servo airplane;
        public static double HOLD = 0.4;
        public static double LAUNCH = 0.8;
        public static void init(HardwareMap hardwareMap){
            airplane = hardwareMap.get(Servo.class, "airplane");
            airplane.setPosition(HOLD);
        }

        public static void launchPlane(){
            airplane.setPosition(LAUNCH);
        }
    }

    public static class AutoPeg{

    }

    @Config
    public static class Macros{
        public static int PAUSE_TIME = 1000;
        public static double POWER = 0.5;

        public static int MIN_BUCKET_PIVOT_POS = 1300;

        private static void SafeSetPivotState(Outtake.PivotState pivotState) throws InterruptedException {
            if(Outtake.getPivotState() != pivotState){
                Outtake.openBlocker();
                Thread.sleep(PAUSE_TIME);

                if((Outtake.getPivotState() == DOWN || pivotState == DOWN) && Slides.getPos() < MIN_BUCKET_PIVOT_POS){
                    Slides.run(MIN_BUCKET_PIVOT_POS, POWER);
                    Thread.sleep(PAUSE_TIME);
                    Outtake.setPivotState(pivotState);
                    Thread.sleep(PAUSE_TIME);
                    Slides.run(1, POWER);
                }else{
                    Outtake.setPivotState(pivotState);
                }
            }
        }
        public static void Intake(){
            try{
                SafeSetPivotState(DOWN);
                Thread.sleep(PAUSE_TIME);
                Outtake.closeBlocker();
                if(!Intake.isActive){Intake.toggle();}
                robotState = INTAKE;
            }catch (Exception ignored){}
        }

        public static void Scoring(){
            try{
                if(Intake.isActive){Intake.toggle();}
                SafeSetPivotState(REST);
                Outtake.setPivotState(REST);
                robotState = SCORING;
            }catch (Exception ignored){}
        }

        public static void Test(){
            try {
                Slides.run(MIN_BUCKET_PIVOT_POS, POWER);
                Outtake.openBlocker();
                Thread.sleep(1000);
            }catch (Exception ignored){}
        }
    }
}
