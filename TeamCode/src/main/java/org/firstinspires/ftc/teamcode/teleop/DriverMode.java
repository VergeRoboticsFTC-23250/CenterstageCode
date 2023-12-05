package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.util.Gamepads.Button.*;
import org.firstinspires.ftc.teamcode.util.Gamepads;
import org.firstinspires.ftc.teamcode.util.Robot;

import java.util.HashMap;

@TeleOp
public class DriverMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        Gamepads.update(gamepad1, gamepad2);
        telemetry.addData("Button", "GP1 Circle open");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            Robot.Chassis.run(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            Robot.Slides.run(-gamepad2.right_stick_y);

            if(Gamepads.onRelease(CIRCLE2, gamepad2.circle)){
                Robot.Intake.toggle();
            }

            if(Gamepads.onRelease(SQUARE2, gamepad2.square)){
                Robot.Hook.toggle(1);
            }

            Gamepads.update(gamepad1, gamepad2);
            telemetry.addData("Slide Posiiton", Robot.Slides.getPos());
            telemetry.addData("Slides Power", -gamepad2.right_stick_y);
            telemetry.update();
        }
    }
}