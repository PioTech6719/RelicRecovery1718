package org.firstinspires.ftc.teamcode.subsystem.sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;


public class DistanceSensorSystem extends Subsystem {

    private DistanceSensor jewelDistanceSensor = null;

    public DistanceSensorSystem(Robot robot, GameMode gameMode) {
        super(robot);
        setGameMode(gameMode);

        init();
    }

    @Override
    public void init() {
        jewelDistanceSensor = Robot.getOpMode().hardwareMap.get(DistanceSensor.class,
                PrometheusConstants.JEWEL_COLOR_DISTANCE_SENSOR);
    }

    @Override
    public void handle(ArrayList<RobotStates> robotStates) {

    }

    @Override
    public void stop() {

    }
}
