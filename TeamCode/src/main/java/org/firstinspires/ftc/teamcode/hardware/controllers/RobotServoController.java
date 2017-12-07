package org.firstinspires.ftc.teamcode.hardware.controllers;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.hardware.RobotHardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.devices.RobotServo;

import java.util.ArrayList;

public class RobotServoController extends RobotHardwareDevice {

    private ArrayList<RobotServo> servos = new ArrayList<>();

    public RobotServoController(@NonNull ServoController servoController, String name) {
        super(servoController, name);
    }

    public RobotServoController(@NonNull ServoController servoController, String name, RobotServo... servos) {
        super(servoController, name);

        for (RobotServo robotServo : servos) {
            addServo(robotServo);
        }
    }

    //TODO: Document and return bool to ensure success
    public void addServo(RobotServo servo) {
        servos.add(servo);
    }

    public void removeServo(RobotServo servo) {
        servos.add(servo);
    }

    public void enableController() {
        ((ServoController) getHardwareDevice()).pwmEnable();
    }

    public void disableController() {
        ((ServoController) getHardwareDevice()).pwmDisable();
    }
}
