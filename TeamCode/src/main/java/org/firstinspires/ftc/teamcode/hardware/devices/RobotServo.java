package org.firstinspires.ftc.teamcode.hardware.devices;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotHardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.controllers.RobotServoController;

public class RobotServo extends RobotHardwareDevice {

    private RobotServoController servoController = null;

    private int port = -1;
    private double currentPosition = -1.0;
    private Servo.Direction currentDirection = null;
    private int SPEED = 1;

    public RobotServo(@NonNull Servo servo, String name) {
        super(servo, name);

        this.port = servo.getPortNumber();
        this.currentPosition = servo.getPosition();
        this.currentDirection = servo.getDirection();
    }

    public RobotServo(@NonNull RobotServoController servoController, @NonNull Servo servo, String name) {
        super(servo, name);

        this.servoController = servoController;
        this.port = servo.getPortNumber();
        this.currentPosition = servo.getPosition();
        this.currentDirection = servo.getDirection();
    }

    public RobotServoController getServoController() {
        return servoController;
    }

    public void setServoController(@NonNull RobotServoController servoController) {
        this.servoController = servoController;
    }

    public int getPort() {
        return port;
    }

    public double getPosition() {
        return ((RobotServo) getHardwareDevice()).getPosition();
    }

    public void setPosition(double position) {
        ((RobotServo) getHardwareDevice()).setPosition(position);
    }

    public void slowSetPosition(double position) {
        slowSetPosition(position, SPEED);
    }

    public void slowSetPosition(double position, double customSpeed) {
        ElapsedTime time = new ElapsedTime();
        time.reset();
        time.startTime();

        if (Robot.getOpMode() instanceof LinearOpMode) {
            while (((LinearOpMode) Robot.getOpMode()).opModeIsActive() && currentPosition - position != 0) {
                //Detect direction to move servo
                if (currentPosition < position) {
                    //Move forward because currentPosition is less than desired position

                    //Detect if speed will overreach desired position
                    if (position - currentPosition < customSpeed) {
                        //If adding speed will overreach, then simply set to position
                        ((RobotServo) getHardwareDevice()).setPosition(position);
                    } else {
                        //Otherwise increment by speed
                        ((RobotServo) getHardwareDevice()).setPosition(currentPosition + customSpeed);
                    }

                    while (((LinearOpMode) Robot.getOpMode()).opModeIsActive() && time.milliseconds() < 100) {
                        ((LinearOpMode) (Robot.getOpMode())).idle();
                    }
                    time.reset();


                } else if (currentPosition > position) {
                    //Move forward because currentPosition is less than desired position

                    //Detect if speed will underreach desired position
                    if (Math.abs(position - currentPosition) < customSpeed) {
                        //If subtracting speed will underreach, then simply set to position
                        ((RobotServo) getHardwareDevice()).setPosition(position);
                        currentPosition = position;
                    } else {
                        //Otherwise decrement by speed
                        ((RobotServo) getHardwareDevice()).setPosition(currentPosition - customSpeed);
                        currentPosition = currentPosition - customSpeed;
                    }

                    while (((LinearOpMode) Robot.getOpMode()).opModeIsActive() && time.milliseconds() < 100) {
                        ((LinearOpMode) (Robot.getOpMode())).idle();
                    }
                    time.reset();
                }
            }
        } else {
            while (currentPosition - position != 0) {
                //Detect direction to move servo
                if (currentPosition < position) {
                    //Move forward because currentPosition is less than desired position

                    //Detect if speed will overreach desired position
                    if (position - currentPosition < customSpeed) {
                        //If adding speed will overreach, then simply set to position
                        ((RobotServo) getHardwareDevice()).setPosition(position);
                    } else {
                        //Otherwise increment by speed
                        ((RobotServo) getHardwareDevice()).setPosition(currentPosition + customSpeed);
                    }

                    while (time.milliseconds() < 100) {
                    }
                    time.reset();


                } else if (currentPosition > position) {
                    //Move forward because currentPosition is less than desired position

                    //Detect if speed will underreach desired position
                    if (Math.abs(position - currentPosition) < customSpeed) {
                        //If subtracting speed will underreach, then simply set to position
                        ((RobotServo) getHardwareDevice()).setPosition(position);
                        currentPosition = position;
                    } else {
                        //Otherwise decrement by speed
                        ((RobotServo) getHardwareDevice()).setPosition(currentPosition - customSpeed);
                        currentPosition = currentPosition - customSpeed;
                    }

                    while (time.milliseconds() < 100) {
                    }
                    time.reset();
                }
            }
        }
    }

    public Servo.Direction getDirection() {
        return currentDirection;
    }

    public void setDirection(Servo.Direction currentDirection) {
        ((RobotServo) getHardwareDevice()).setDirection(currentDirection);
        this.currentDirection = currentDirection;
    }
}
