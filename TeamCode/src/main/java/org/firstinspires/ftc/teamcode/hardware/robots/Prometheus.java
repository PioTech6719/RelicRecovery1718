package org.firstinspires.ftc.teamcode.hardware.robots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.controllers.RobotMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.RobotServoController;
import org.firstinspires.ftc.teamcode.hardware.devices.RobotMotor;
import org.firstinspires.ftc.teamcode.hardware.devices.RobotServo;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;


public class Prometheus extends Robot {

    private RobotMotorController leftDriveMotors = null;
    private RobotMotorController rightDriveMotors = null;
    private RobotMotorController liftMotorController = null;

    private RobotServoController glyphGrabberUpperServos = null;
    private RobotServoController glyphGrabberLowerServos = null;

    private RobotMotor leftFrontMotor = null;
    private RobotMotor leftRearMotor = null;
    private RobotMotor rightFrontMotor = null;
    private RobotMotor rightRearMotor = null;
    private RobotMotor liftMotor = null;

    private RobotServo upperLeftServo = null;
    private RobotServo upperRightServo = null;
    private RobotServo lowerLeftServo = null;
    private RobotServo lowerRightServo = null;

    private ColorSensor jewelColorSensor = null;
    private DistanceSensor jewelDistanceSensor = null;
    private BNO055IMU gyroSensor = null;

    public Prometheus(OpMode opMode) {
        super(opMode);
    }

    private void calibrateGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyroSensor.initialize(parameters);
    }

    private boolean connectDevices(GameMode gameMode) {
        leftDriveMotors = addMotorController("Left Drive Motors");
        rightDriveMotors = addMotorController("Right Drive Motors");
        liftMotorController = addMotorController("Lift Motor Controller");

        glyphGrabberUpperServos = addServoController("Glyph Upper Servos");
        glyphGrabberLowerServos = addServoController("Gylph Lower Servos");

        leftFrontMotor = addMotor("Left Front Motor");
        leftRearMotor = addMotor("Left Rear Motor");
        rightFrontMotor = addMotor("Right Front Motor");
        rightRearMotor = addMotor("Right Rear Motor");

        liftMotor = addMotor("Lift Motor");

        upperLeftServo = addServo(glyphGrabberUpperServos, "Upper Left Servo");
        upperRightServo = addServo(glyphGrabberUpperServos, "Upper Right Servo");
        lowerLeftServo = addServo(glyphGrabberLowerServos, "Lower Left Servo");
        lowerRightServo = addServo(glyphGrabberUpperServos, "Lower Right Servo");

        if (gameMode.equals(GameMode.AUTO)) {
            jewelColorSensor = getOrNull(getOpMode().hardwareMap.colorSensor, "Jewel Color Sensor");
            jewelDistanceSensor = getOpMode().hardwareMap.get(DistanceSensor.class,
                    "Jewel Distance Sensor");

            gyroSensor = getOpMode().hardwareMap.get(BNO055IMU.class, "imu");
        }

        //Return whether or not all devices are connected or some are null
        return true;
    }

    public void init(GameMode gameMode) {
        connectDevices(gameMode);
        setMotorDirections();

        if (gameMode.equals(GameMode.AUTO)) {
            calibrateGyro();
        }
    }

    //TODO: Catch exception and prevent error out if motors are null
    private void setMotorDirections() {
        ((DcMotor) leftFrontMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.FORWARD);
        ((DcMotor) leftRearMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.FORWARD);
        ((DcMotor) rightFrontMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.REVERSE);
        ((DcMotor) rightRearMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.REVERSE);

        ((DcMotor) liftMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
