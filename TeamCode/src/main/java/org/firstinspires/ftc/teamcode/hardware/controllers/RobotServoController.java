package org.firstinspires.ftc.teamcode.hardware.controllers;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.hardware.devices.RobotServo;

import java.util.ArrayList;

public class RobotServoController {

    private ServoController servoController = null;
    private ArrayList<RobotServo> servos = new ArrayList<>();

    public RobotServoController(@NonNull ServoController servoController) {
        this.servoController = servoController;
    }

    public RobotServoController(@NonNull ServoController servoController, RobotServo... servos) {
        this.servoController = servoController;

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
        servoController.pwmEnable();
    }

    public void disableController() {
        servoController.pwmDisable();
    }
}
