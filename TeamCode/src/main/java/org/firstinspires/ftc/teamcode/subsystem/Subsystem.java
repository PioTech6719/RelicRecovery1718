package org.firstinspires.ftc.teamcode.subsystem;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;
import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;

public abstract class Subsystem {

    private Robot robot = null;

    private GameMode gameMode = null;

    public Subsystem(Robot robot) {
        this.robot = robot;
    }

    public abstract void init();

    public abstract void handle(ArrayList<RobotStates> robotStates);

    public abstract void stop();

    public Robot getRobot() {
        return robot;
    }

    public GameMode getGameMode() {
        return gameMode;
    }

    public void setGameMode(GameMode gameMode) {
        this.gameMode = gameMode;
    }
}
