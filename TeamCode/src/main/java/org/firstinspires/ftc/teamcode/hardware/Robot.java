package org.firstinspires.ftc.teamcode.hardware;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.controllers.RobotMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.RobotServoController;
import org.firstinspires.ftc.teamcode.hardware.devices.RobotMotor;
import org.firstinspires.ftc.teamcode.hardware.devices.RobotServo;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;

import java.util.ArrayList;
import java.util.Map;

public abstract class Robot {

    private static OpMode opMode = null;

    private ArrayList<RobotMotorController> robotMotorControllers = new ArrayList<>();
    private ArrayList<RobotServoController> robotServoControllers = new ArrayList<>();
    private ArrayList<RobotMotor> robotMotors = new ArrayList<>(); //TODO: Add driveMotors arraylist for use in movementControllers. movementControllers needs a base class and also must not be specific to drivetrain also have setdirection not be specific to drivetrain
    private ArrayList<RobotServo> robotServos = new ArrayList<>();

    public Robot(OpMode opMode) {
        Robot.opMode = opMode;
    }

    public static OpMode getOpMode() {
        return opMode;
    }

    public ArrayList<RobotMotorController> getRobotMotorControllers() {
        return robotMotorControllers;
    }

    public ArrayList<RobotServoController> getRobotServoControllers() {
        return robotServoControllers;
    }

    public ArrayList<RobotMotor> getRobotMotors() {
        return robotMotors;
    }

    public ArrayList<RobotServo> getRobotServos() {
        return robotServos;
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

    private RobotHardwareDevice getDevice(Class deviceType, String deviceName) {
        if (deviceType.equals(DcMotorController.class)) {
            return new RobotMotorController(getOrNull(getOpMode().hardwareMap.dcMotorController, deviceName), deviceName);
        } else if (deviceType.equals(ServoController.class)) {
            return new RobotServoController(getOrNull(getOpMode().hardwareMap.servoController, deviceName), deviceName);
        } else if (deviceType.equals(DcMotor.class)) {
            return new RobotMotor(getOrNull(getOpMode().hardwareMap.dcMotor, deviceName), deviceName);
        } else if (deviceType.equals(Servo.class)) {
            return new RobotServo(getOrNull(getOpMode().hardwareMap.servo, deviceName), deviceName);
        }

        return null;
    }

    public abstract void init(GameMode gameMode);

    public RobotMotorController addMotorController(String name) {
        robotMotorControllers.add((RobotMotorController) getDevice(DcMotorController.class, name));
        return (RobotMotorController) getDevice(DcMotorController.class, name);
    }

    public RobotServoController addServoController(String name) {
        robotServoControllers.add((RobotServoController) getDevice(ServoController.class, name));
        return (RobotServoController) getDevice(ServoController.class, name);
    }

    public RobotMotor addMotor(String name) {
        robotMotors.add((RobotMotor) getDevice(DcMotor.class, name));
        return (RobotMotor) getDevice(DcMotor.class, name);
    }

    public RobotMotor addMotor(RobotMotorController motorController, String name) {
        RobotMotor robotMotor = (RobotMotor) getDevice(DcMotor.class, name);
        robotMotor.setMotorController(motorController);
        robotMotors.add(robotMotor);
        return robotMotor;
    }

    public RobotServo addServo(String name) {
        robotServos.add((RobotServo) getDevice(Servo.class, name));
        return (RobotServo) getDevice(Servo.class, name);
    }

    public RobotServo addServo(RobotServoController servoController, String name) {
        RobotServo robotServo = (RobotServo) getDevice(Servo.class, name);
        robotServo.setServoController(servoController);
        robotServos.add(robotServo);
        return robotServo;
    }

    //TODO: Convert to extrqact DcMotor from RobotMotor and input RobotMotor to keep opmode classes clean
    public void setMotorsMode(@NonNull DcMotor.RunMode runMode, @NonNull DcMotor... motors) {
        resetMotors(motors);

        for (DcMotor motor : motors) {
            motor.setMode(runMode);
        }
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
