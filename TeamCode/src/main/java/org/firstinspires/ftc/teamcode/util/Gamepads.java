package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.Gamepads.Button.*;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

public class Gamepads{
    public enum Button {
        CIRCLE1,
        SQUARE1,
        TRIANGLE1,
        CROSS1,
        CIRCLE2,
        SQUARE2,
        TRIANGLE2,
        CROSS2,
    }
    private static HashMap<Button, Boolean> keyStates = new HashMap<Button, Boolean>();

    public static void update (Gamepad gp1, Gamepad gp2){
        keyStates.put(CIRCLE1, gp1.circle);
        keyStates.put(SQUARE1, gp1.square);
        keyStates.put(TRIANGLE1, gp1.triangle);
        keyStates.put(CROSS1, gp1.cross);

        keyStates.put(CIRCLE2, gp2.circle);
        keyStates.put(SQUARE2, gp2.square);
        keyStates.put(TRIANGLE2, gp2.triangle);
        keyStates.put(CROSS2, gp2.cross);
    }

    public static boolean onRelease(Button key, boolean currentState){
        boolean lastState = Boolean.TRUE.equals(keyStates.get(key));
        return lastState && !currentState;
    }
}