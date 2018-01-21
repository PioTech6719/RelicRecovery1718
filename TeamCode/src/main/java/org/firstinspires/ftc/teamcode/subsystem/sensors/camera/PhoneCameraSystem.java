package org.firstinspires.ftc.teamcode.subsystem.sensors.camera;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;


public class PhoneCameraSystem extends Subsystem {

    private VuforiaSystem vuforiaSystem = null;
    private boolean visibleView = true;
    private boolean isVuforiaInitialized;

    public PhoneCameraSystem(Robot robot) {
        super(robot);
        init();
    }

    @Override
    public void init() {
        vuforiaSystem = new VuforiaSystem(getRobot(), visibleView);
    }

    @Override
    public void handle(ArrayList<RobotStates> robotStates) {
        vuforiaSystem.handle(robotStates);
    }

    @Override
    public void stop() {
        vuforiaSystem.stop();
    }

    public VuforiaSystem getVuforiaSystem() {
        return vuforiaSystem;
    }

    public void toggleVuforia(boolean status) {
        if (status && !isVuforiaInitialized) {
            vuforiaSystem.init();
            isVuforiaInitialized = true;
        } else {
            vuforiaSystem.stop();
        }
    }
}
