package org.firstinspires.ftc.teamcode.hardware.gamepads;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;

public abstract class GamepadConfig {

    private CustomGamepad gamepad1 = null;
    private CustomGamepad gamepad2 = null;

    public GamepadConfig(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = new CustomGamepad(gamepad1);
        this.gamepad2 = new CustomGamepad(gamepad2);

        initGamepads();
    }

    public CustomGamepad getGamepad1() {
        return gamepad1;
    }

    public CustomGamepad getGamepad2() {
        return gamepad2;
    }

    private void initGamepads() {
        gamepad1.getGamepad().setJoystickDeadzone(.1f);
        gamepad2.getGamepad().setJoystickDeadzone(.1f);
    }

    public abstract ArrayList<RobotStates> getStates();
}
