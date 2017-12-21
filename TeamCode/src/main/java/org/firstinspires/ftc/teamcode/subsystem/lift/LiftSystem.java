package org.firstinspires.ftc.teamcode.subsystem.lift;

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

public class LiftSystem extends Subsystem {

    private RobotMotorController liftMotorController = null;
    private RobotMotor liftMotor = null;

    public LiftSystem(Robot robot, GameMode gameMode) {
        super(robot);
        setGameMode(gameMode);

        init();
    }

    @Override
    public void init() {
        liftMotorController = getRobot().addMotorController(PrometheusConstants.LIFT_DRIVE_MOTOR_CONTROLLER);
        liftMotor = getRobot().addMotor(liftMotorController, PrometheusConstants.LIFT_MOTOR);

        setDirection();
        setMotorMode();
    }

    @Override
    public void handle(ArrayList<RobotStates> robotStates) {

    }

    @Override
    public void stop() {

    }

    private void setDirection() {
        ((DcMotor) liftMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void setMotorMode() {
        ((DcMotor) liftMotor.getHardwareDevice()).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
