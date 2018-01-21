package org.firstinspires.ftc.teamcode.terminators;

import org.firstinspires.ftc.teamcode.subsystem.sensors.LightSensorSystem;

public abstract class LightChangeTerminator implements Terminator {

    private LightSensorSystem lightSensorSystem;
    private double currentLight;
    private double THRESHOLD = 0.13;

    public LightChangeTerminator(LightSensorSystem lightSensorSystem, double THRESHOLD) {
        this.lightSensorSystem = lightSensorSystem;
        this.currentLight = -1;
        this.THRESHOLD = THRESHOLD;
    }

    @Override
    public boolean shouldTerminate() {
        if (currentLight == -1) {
            currentLight = lightSensorSystem.getLight();
        }
        return Math.abs(currentLight - lightSensorSystem.getLight()) > THRESHOLD;
    }

}
