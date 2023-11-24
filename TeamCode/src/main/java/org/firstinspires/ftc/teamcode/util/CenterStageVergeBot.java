package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CenterStageVergeBot {
    DcMotor rightSlideMotor;
    DcMotor leftSlideMotor;

    static final int SLIDE_MAX = 2000;

    public CenterStageVergeBot(HardwareMap hardwareMap){
        rightSlideMotor = hardwareMap.dcMotor.get("rightSlideMotor");
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor = hardwareMap.dcMotor.get("leftSlideMotor");
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runSlidesTo(int position, int power){
        if(position < 0 || position > SLIDE_MAX) return;
        rightSlideMotor.setTargetPosition(position);
        leftSlideMotor.setTargetPosition(position);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }
}
