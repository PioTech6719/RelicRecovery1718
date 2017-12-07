package org.firstinspires.ftc.teamcode.hardware.controllers;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorController;

import org.firstinspires.ftc.teamcode.hardware.RobotHardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.devices.RobotMotor;

import java.util.ArrayList;

public class RobotMotorController extends RobotHardwareDevice {

    private ArrayList<RobotMotor> motors = new ArrayList<>();

    public RobotMotorController(@NonNull DcMotorController motorController, String name, RobotMotor... motors) {
        super(motorController, name);

        for (RobotMotor motor : motors) {
            this.motors.add(motor);
        }
    }
}
