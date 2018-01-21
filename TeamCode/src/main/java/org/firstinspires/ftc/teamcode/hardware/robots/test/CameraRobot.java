package org.firstinspires.ftc.teamcode.hardware.robots.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;
import org.firstinspires.ftc.teamcode.subsystem.sensors.camera.PhoneCameraSystem;


public class CameraRobot extends Robot {

    private PhoneCameraSystem phoneCameraSystem;

    public CameraRobot(OpMode opMode) {
        super(opMode);
    }

    private boolean connectDevices(GameMode gameMode) {
        phoneCameraSystem = (PhoneCameraSystem) addSubsystem(new PhoneCameraSystem(this));

        //Return whether or not all devices are connected or some are null
        return true;
    }

    @Override
    public void init(GameMode gameMode) {
        connectDevices(gameMode);
    }
}
