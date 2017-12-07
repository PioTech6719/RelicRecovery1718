package org.firstinspires.ftc.teamcode.hardware.devices;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RobotHardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.controllers.RobotMotorController;

public class RobotMotor extends RobotHardwareDevice {

    private RobotMotorController motorController = null;

    public RobotMotor(@NonNull DcMotor motor, @NonNull String name) {
        super(motor, name);
        //Maybe use .getDeviceName from DcMotor class instead?
    }

    public RobotMotor(@NonNull RobotMotorController motorController, @NonNull DcMotor motor, @NonNull String name) {
        super(motor, name);
        //Maybe use .getDeviceName from DcMotor class instead?
        this.motorController = motorController;
    }

    public RobotMotorController getMotorController() {
        return motorController;
    }

    public void setMotorController(@NonNull RobotMotorController motorController) {
        this.motorController = motorController;
    }

    //TODO: Implement
    public boolean isStalling() {
        return false;
    }
}
