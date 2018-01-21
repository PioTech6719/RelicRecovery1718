package org.firstinspires.ftc.teamcode.subsystem.sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;

public class ColorSensorSystem extends Subsystem {

    //Hue, Saturation, Value
    float hsvValues[] = {0F, 0F, 0F};
    private ColorSensor colorSensor = null;

    public ColorSensorSystem(Robot robot, GameMode gameMode) {
        super(robot);
        setGameMode(gameMode);

        init();
    }

    @Override
    public void init() {
        colorSensor = getRobot().getOrNull(Robot.getOpMode().hardwareMap.colorSensor,
                PrometheusConstants.JEWEL_COLOR_DISTANCE_SENSOR);
    }

    @Override
    public void handle(ArrayList<RobotStates> robotStates) {

    }

    @Override
    public void stop() {

    }

    private void getColor() {
        //Could use RGBToHSV or aRGB to return color-int
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
    }

    public double getHue() {
        getColor();
        return hsvValues[0];
    }

    public double getSaturation() {
        getColor();
        return hsvValues[1];
    }

    public double getValue() {
        getColor();
        return hsvValues[2];
    }

    public double getRed() {
        return colorSensor.red();
    }

    public double getBlue() {
        return colorSensor.blue();
    }

    public double getGreen() {
        return colorSensor.green();
    }
}
