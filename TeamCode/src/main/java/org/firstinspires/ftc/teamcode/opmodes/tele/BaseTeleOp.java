package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;

public abstract class BaseTeleOp extends OpMode {

    private final GameMode GAME_MODE = GameMode.TELEOP;
    private Robot robot = null;

    public BaseTeleOp(Robot robot) {
        this.robot = robot;
    }

    public void init() {
        robot.init(GAME_MODE);
    }

    //TODO: add method to manage telemetry mb one method append() and one method write() append adds
}
