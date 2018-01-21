package org.firstinspires.ftc.teamcode.subsystem.sensors;

import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;

public class LightSensorSystem extends Subsystem {

    private LightSensor lightSensor = null;
    private boolean ledStatus = true;

    public LightSensorSystem(Robot robot) {
        super(robot);

        init();
    }

    @Override
    public void init() {
        lightSensor = Robot.getOpMode().hardwareMap.get(LightSensor.class, PrometheusConstants.LIGHT_SENSOR);
        lightSensor.enableLed(ledStatus);
    }

    @Override
    public void handle(ArrayList<RobotStates> robotStates) {

    }

    @Override
    public void stop() {

    }

    public void enableLed(boolean ledStatus) {
        this.ledStatus = ledStatus;
        lightSensor.enableLed(ledStatus);
    }

    public double getLight() {
        return lightSensor.getLightDetected();
    }

    public double getRawLight() {
        return lightSensor.getRawLightDetected();
    }
}
