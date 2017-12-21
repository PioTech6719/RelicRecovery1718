package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;
import org.firstinspires.ftc.teamcode.hardware.controllers.RobotMotorController;
import org.firstinspires.ftc.teamcode.hardware.devices.RobotMotor;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;

public class DriveSystem extends Subsystem {

    private RobotMotorController leftDriveMotors = null;
    private RobotMotorController rightDriveMotors = null;

    private RobotMotor leftFrontMotor = null;
    private RobotMotor leftRearMotor = null;
    private RobotMotor rightFrontMotor = null;
    private RobotMotor rightRearMotor = null;

    public DriveSystem(Robot robot, GameMode gameMode) {
        super(robot);
        setGameMode(gameMode);

        init();
    }

    @Override
    public void init() {
        leftDriveMotors = getRobot().addMotorController(PrometheusConstants.LEFT_DRIVE_MOTOR_CONTROLLER);
        rightDriveMotors = getRobot().addMotorController(PrometheusConstants.RIGHT_DRIVE_MOTOR_CONTROLLER);

        leftFrontMotor = getRobot().addMotor(leftDriveMotors, PrometheusConstants.LEFT_FRONT_DRIVE_MOTOR);
        leftRearMotor = getRobot().addMotor(leftDriveMotors, PrometheusConstants.LEFT_REAR_DRIVE_MOTOR);

        rightFrontMotor = getRobot().addMotor(rightDriveMotors, PrometheusConstants.RIGHT_FRONT_DRIVE_MOTOR);
        rightRearMotor = getRobot().addMotor(rightDriveMotors, PrometheusConstants.RIGHT_REAR_DRIVE_MOTOR);

        setDirections();
        setDriveMotorMode();
    }

    @Override
    public void handle(ArrayList<RobotStates> robotStates) {

    }

    @Override
    public void stop() {
        ((DcMotor) leftFrontMotor.getHardwareDevice()).setPower(0);
        ((DcMotor) leftRearMotor.getHardwareDevice()).setPower(0);
        ((DcMotor) rightFrontMotor.getHardwareDevice()).setPower(0);
        ((DcMotor) rightRearMotor.getHardwareDevice()).setPower(0);
    }

    private void setDirections() {
        ((DcMotor) leftFrontMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.FORWARD);
        ((DcMotor) leftRearMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.FORWARD);
        ((DcMotor) rightFrontMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.REVERSE);
        ((DcMotor) rightRearMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setDriveMotorMode() {
        if (getGameMode().equals(GameMode.AUTO)) {
            getRobot().setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER,
                    (DcMotor) leftFrontMotor.getHardwareDevice(),
                    (DcMotor) leftRearMotor.getHardwareDevice(),
                    (DcMotor) rightFrontMotor.getHardwareDevice(),
                    (DcMotor) rightRearMotor.getHardwareDevice());
        } else if (getGameMode().equals(GameMode.TELEOP)) {
            getRobot().setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                    (DcMotor) leftFrontMotor.getHardwareDevice(),
                    (DcMotor) leftRearMotor.getHardwareDevice(),
                    (DcMotor) rightFrontMotor.getHardwareDevice(),
                    (DcMotor) rightRearMotor.getHardwareDevice());
        }
    }
}
