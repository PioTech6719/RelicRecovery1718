package org.firstinspires.ftc.teamcode.hardware.robots.old;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class JewelProvider {

    //Calculated color values using colorizer.org
    private final double MIN_BLUE_RANGE = 150;
    private final double MAX_BLUE_RANGE = 260;
    private final double MIN_RED_RANGE = 0;
    private final double MAX_RED_RANGE = 20;
    float hsvValues[] = {0F, 0F, 0F}; //Hue, Saturation, Value
    private ColorSensor colorSensor = null;

    public JewelProvider(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
        init();
    }

    public void init() {
        colorSensor.enableLed(true);
    }

    public Jewel getJewel() {
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        if (hsvValues[0] >= MIN_BLUE_RANGE && hsvValues[0] <= MAX_BLUE_RANGE) { //Blue Jewel
            return Jewel.BLUE;
        }

        if (hsvValues[0] >= MIN_RED_RANGE && hsvValues[0] <= MAX_RED_RANGE) { //Red Jewel
            return Jewel.RED;
        }

        return Jewel.NONE;
    }

    public enum Jewel {
        BLUE, RED, NONE
    }
}
