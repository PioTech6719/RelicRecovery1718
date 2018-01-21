package org.firstinspires.ftc.teamcode.terminators;

import org.firstinspires.ftc.teamcode.subsystem.sensors.ColorSensorSystem;

public class ColorChangeTerminator implements Terminator {

    private ColorSensorSystem colorSensorSystem;
    private double currentHue;
    private double THRESHOLD = 25;

    public ColorChangeTerminator(ColorSensorSystem colorSensorSystem, double THRESHOLD) {
        this.colorSensorSystem = colorSensorSystem;
        currentHue = -1;
        this.THRESHOLD = THRESHOLD;
    }

    @Override
    public boolean shouldTerminate() {
        if (currentHue == -1) {
            currentHue = colorSensorSystem.getHue();
        }
        return Math.abs(currentHue - colorSensorSystem.getHue()) > THRESHOLD;
    }
}
