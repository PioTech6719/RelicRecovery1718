package org.firstinspires.ftc.teamcode.subsystem.sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;

public class ColorSensorSystem extends Subsystem {

    private ColorSensor jewelColorSensor = null;

    public ColorSensorSystem(Robot robot, GameMode gameMode) {
        super(robot);
        setGameMode(gameMode);

        init();
    }

    @Override
    public void init() {
        jewelColorSensor = getRobot().getOrNull(Robot.getOpMode().hardwareMap.colorSensor,
                PrometheusConstants.JEWEL_COLOR_DISTANCE_SENSOR);
    }

    @Override
    public void handle(ArrayList<RobotStates> robotStates) {

    }

    @Override
    public void stop() {

    }
}
