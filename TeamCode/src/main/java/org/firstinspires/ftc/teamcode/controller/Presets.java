package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.Timer;

public class Presets {
    public static CustomController driverPreset(Gamepad gamepad1) {
        return new CustomController(gamepad1,
                gamepad -> gamepad.left_stick_x,
                gamepad -> gamepad.left_stick_y,
                gamepad -> gamepad.right_stick_x,
                gamepad -> 0,
                Gamepad::leftBumperWasPressed,
                gamepad -> gamepad.right_trigger - gamepad.left_trigger
                );
    }
    public static CustomController technicianPreset(Gamepad gamepad2) {
        return  new CustomController(gamepad2,
                    gamepad -> gamepad.left_trigger * Timer.DTime(),
                gamepad -> gamepad.right_trigger * Timer.DTime(),
                Gamepad::circleWasPressed,      ///Auto park
                Gamepad::dpadLeftWasPressed,
                Gamepad::dpadRightWasPressed,
                Gamepad::triangleWasPressed,
                Gamepad::dpadUpWasPressed,
                Gamepad::dpadDownWasPressed,
                Gamepad::squareWasPressed
                );
    }


}
