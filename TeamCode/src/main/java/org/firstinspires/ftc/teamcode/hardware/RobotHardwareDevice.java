package org.firstinspires.ftc.teamcode.hardware;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareDevice;

public class RobotHardwareDevice {

    private HardwareDevice hardwareDevice = null;
    private String name = "";


    public RobotHardwareDevice(@NonNull HardwareDevice hardwareDevice, @NonNull String name) {
        this.hardwareDevice = hardwareDevice;
        this.name = name;
    }

    public HardwareDevice getHardwareDevice() {
        return hardwareDevice;
    }

    public String getName() {
        return name;
    }
}
