package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;

public class GyroProvider {

    private BNO055IMU imu = null;

    public GyroProvider() {
        imu = Robot.getOpMode().hardwareMap.get(BNO055IMU.class, PrometheusConstants.GYRO_SENSOR);
    }
}
