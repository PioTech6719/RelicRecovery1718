package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.gamepads.GamepadConfig;
import org.firstinspires.ftc.teamcode.match.Alliance;
import org.firstinspires.ftc.teamcode.match.AllianceProperties;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;


public abstract class BaseAutoOp extends LinearOpMode {

    private final GameMode GAME_MODE = GameMode.AUTO;
    private Alliance alliance;
    private AllianceProperties allianceProperties;
    private Robot robot = null;
    private GamepadConfig gamepadConfig = null;

    public abstract void initializeClass();

    @Override
    public void runOpMode() throws InterruptedException {
        initializeClass();
        robot.init(GAME_MODE);
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }

    public void setGamepadConfig(GamepadConfig gamepadConfig) {
        this.gamepadConfig = gamepadConfig;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        this.allianceProperties = new AllianceProperties(alliance);
    }

    public AllianceProperties getAllianceProperties() {
        return allianceProperties;
    }
}
