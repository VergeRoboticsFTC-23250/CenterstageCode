package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.HashMap;

public class Robot {
    public static void init(HardwareMap hardwareMap){
        Chassis.init(hardwareMap);
        Slides.init(hardwareMap);
        Hook.init(hardwareMap);
        Intake.init(hardwareMap);
        Outtake.init(hardwareMap);
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
        private static DcMotor leftSlideMotor;
        private static DcMotor rightSlideMotor;

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
        }

        public static void home(int pow){
            run(0, pow);
            rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }

    @Config
    public static class Hook{
        public static int maxPos = 1000;
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
            hookMotor.setTargetPosition(isExtended? maxPos : 0);
            hookMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hookMotor.setPower(pow);
        }
    }

    @Config
    public static class Intake{
        private static DcMotor motor;
        public static Servo pivot;

        private static boolean isActive = false;
        public static double speed = 0.5;
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
    }

    @Config
    public static class Outtake{
        private static Servo pivot;
        private static Servo ramp;

        public static double pivotMax = 1;
        public static double pivotMin = 0;
        private static boolean isActive = false;

        public static double rampMax = 1;
        public static double rampRest = 0.5;
        public static double rampMin = 0;

        public static void init(HardwareMap hardwareMap){
            pivot = hardwareMap.get(Servo.class, "outtakePivot");
            updatePivot();
            ramp = hardwareMap.get(Servo.class, "outtakeRamp");
            ramp.setPosition(rampRest);
        }

        private static void updatePivot(){
            pivot.setPosition(isActive? pivotMax : pivotMin);
        }

        public static void toggle(){
            isActive = !isActive;
            updatePivot();
        }

        public static void setRamp(double pos){
            ramp.setPosition(pos);
        }
    }
}
