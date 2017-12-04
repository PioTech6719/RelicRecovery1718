package org.firstinspires.ftc.teamcode.hardware;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.controllers.RobotServoController;

import java.util.ArrayList;
import java.util.Map;

public class Robot {

    private static OpMode opMode = null;
    private static ArrayList<RobotServoController> robotServoControllers = new ArrayList<>();
    private static ArrayList<DcMotor> robotMotors = new ArrayList<>();

    public Robot(OpMode opMode) {
        Robot.opMode = opMode;
    }

    public static OpMode getOpMode() {
        return opMode;
    }

    public static ArrayList<RobotServoController> getRobotServoControllers() {
        return robotServoControllers;
    }

    /**
     * Get the value associated with an id and instead of raising an error return null and log it
     *
     * @param map  the hardware map from the HardwareMap
     * @param name The ID in the hardware map
     * @param <T>  the type of hardware map
     * @return the hardware device associated with the name
     */
    protected <T extends HardwareDevice> T getOrNull(@NonNull HardwareMap.DeviceMapping<T> map,
                                                     String name) {
        for (Map.Entry<String, T> item : map.entrySet()) {
            if (!item.getKey().equalsIgnoreCase(name)) {
                continue;
            }
            return item.getValue();
        }
        opMode.telemetry.addLine("ERROR: " + name + " not found!");
        RobotLog.e("ERROR: " + name + " not found!");
        return null;
    }

    public void addMotor(String name) {
        robotMotors.add(getOrNull(getOpMode().hardwareMap.dcMotor, name));
    }

    //TODO: Implement
    public void removeMotor(String name) {

    }

    /**
     * Waits for all the motors to have zero position and if it is not zero tell it to reset
     *
     * @param motors motors to reset
     */
    public void resetMotors(@NonNull DcMotor... motors) {
        boolean notReset = true;
        while (notReset) {
            boolean allReset = true;
            for (DcMotor motor : motors) {
                motor.setPower(0);
                if (motor.getCurrentPosition() == 0) {
                    continue;
                }
                allReset = false;
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            notReset = !allReset;
        }
        for (DcMotor motor : motors) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Supplies power to respective servo controllers
     *
     * @param servoControllers servo controllers to power on
     */
    public void enableServoControllers(@NonNull ServoController... servoControllers) {
        for (ServoController servoController : servoControllers) {
            servoController.pwmEnable();
        }
    }

    /**
     * Disables power to respective servo controllers
     *
     * @param servoControllers servo controllers to power off
     */
    public void disableServoControllers(@NonNull ServoController... servoControllers) {
        for (ServoController servoController : servoControllers) {
            servoController.pwmDisable();
        }
    }

}
