package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.gamepads.GamepadConfig;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;

public abstract class BaseTeleOp extends OpMode {

    private GameMode GAME_MODE = GameMode.TELEOP;
    private Robot robot = null;
    private GamepadConfig gamepadConfig = null;

    public abstract void initializeClass();

    @Override
    public void init() {
        initializeClass();
        robot.init(GAME_MODE);
    }

    public void setGAME_MODE(GameMode GAME_MODE) {
        this.GAME_MODE = GAME_MODE;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }

    public void setGamepadConfig(GamepadConfig gamepadConfig) {
        this.gamepadConfig = gamepadConfig;
    }

    //TODO: add method to manage telemetry mb one method append() and one method write() append adds
}
