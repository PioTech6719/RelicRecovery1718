package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;
import org.firstinspires.ftc.teamcode.hardware.controllers.RobotMotorController;
import org.firstinspires.ftc.teamcode.hardware.devices.RobotMotor;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.movement.MovementController;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.movement.encoder.EncoderGyroMovementController;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;

public class DriveSystem extends Subsystem {

    private RobotMotorController leftDriveMotors = null;
    private RobotMotorController rightDriveMotors = null;

    private RobotMotor leftFrontMotor = null;
    private RobotMotor leftRearMotor = null;
    private RobotMotor rightFrontMotor = null;
    private RobotMotor rightRearMotor = null;

    private MovementController movementController = null; //use multiple systems at once or nah

    private double DRIVE_GEAR_REDUCTION = 2.0;
    private double AUTO_SPEED = 0.5;

    //TODO: Add null checks everywhere

    /**
     * @param robot
     * @param gameMode
     */
    public DriveSystem(Robot robot, GameMode gameMode) {
        super(robot);
        setGameMode(gameMode);

        init();
    }

    @Override
    public void init() {
//        leftDriveMotors = getRobot().addMotorController(PrometheusConstants.LEFT_DRIVE_MOTOR_CONTROLLER);
//        rightDriveMotors = getRobot().addMotorController(PrometheusConstants.RIGHT_DRIVE_MOTOR_CONTROLLER);

        leftFrontMotor = getRobot().addMotor(leftDriveMotors, PrometheusConstants.LEFT_FRONT_DRIVE_MOTOR);
        leftRearMotor = getRobot().addMotor(leftDriveMotors, PrometheusConstants.LEFT_REAR_DRIVE_MOTOR);

        rightFrontMotor = getRobot().addMotor(rightDriveMotors, PrometheusConstants.RIGHT_FRONT_DRIVE_MOTOR);
        rightRearMotor = getRobot().addMotor(rightDriveMotors, PrometheusConstants.RIGHT_REAR_DRIVE_MOTOR);

        setDirections();
        setDriveMotorMode();
        setMovementController(new EncoderGyroMovementController(
                getRobot(),
                this,
                DRIVE_GEAR_REDUCTION,
                AUTO_SPEED,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }

    public RobotMotorController getLeftDriveMotors() {
        return leftDriveMotors;
    }

    public RobotMotorController getRightDriveMotors() {
        return rightDriveMotors;
    }

    public RobotMotor getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public RobotMotor getLeftRearMotor() {
        return leftRearMotor;
    }

    public RobotMotor getRightFrontMotor() {
        return rightFrontMotor;
    }

    public RobotMotor getRightRearMotor() {
        return rightRearMotor;
    }

    public MovementController getMovementController() {
        return movementController;
    }

    public void setMovementController(MovementController movementController) {
        this.movementController = movementController;
    }

    public void resetEncoders() {
        getRobot().resetMotors(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
    }

    @Override
    public void handle(ArrayList<RobotStates> robotStates) {
        if (robotStates.contains(RobotStates.DRIVE_FORWARD)) {
            //TODO: later convert to methods for each direction and takes input if input is null then infinite
            ((DcMotor) leftFrontMotor.getHardwareDevice()).setPower(Constants.DEFAULT_SPEED);
            ((DcMotor) leftRearMotor.getHardwareDevice()).setPower(Constants.DEFAULT_SPEED);
            ((DcMotor) rightFrontMotor.getHardwareDevice()).setPower(Constants.DEFAULT_SPEED);
            ((DcMotor) rightRearMotor.getHardwareDevice()).setPower(Constants.DEFAULT_SPEED);
        }

        if (robotStates.contains(RobotStates.DRIVE_REVERSE)) {
            ((DcMotor) leftFrontMotor.getHardwareDevice()).setPower(-Constants.DEFAULT_SPEED);
            ((DcMotor) leftRearMotor.getHardwareDevice()).setPower(-Constants.DEFAULT_SPEED);
            ((DcMotor) rightFrontMotor.getHardwareDevice()).setPower(-Constants.DEFAULT_SPEED);
            ((DcMotor) rightRearMotor.getHardwareDevice()).setPower(-Constants.DEFAULT_SPEED);
        }

        if (robotStates.contains(RobotStates.DRIVE_LEFT)) {
            ((DcMotor) leftFrontMotor.getHardwareDevice()).setPower(-Constants.DEFAULT_SPEED);
            ((DcMotor) leftRearMotor.getHardwareDevice()).setPower(Constants.DEFAULT_SPEED);
            ((DcMotor) rightFrontMotor.getHardwareDevice()).setPower(Constants.DEFAULT_SPEED);
            ((DcMotor) rightRearMotor.getHardwareDevice()).setPower(-Constants.DEFAULT_SPEED);
        }

        if (robotStates.contains(RobotStates.DRIVE_RIGHT)) {
            ((DcMotor) leftFrontMotor.getHardwareDevice()).setPower(Constants.DEFAULT_SPEED);
            ((DcMotor) leftRearMotor.getHardwareDevice()).setPower(-Constants.DEFAULT_SPEED);
            ((DcMotor) rightFrontMotor.getHardwareDevice()).setPower(-Constants.DEFAULT_SPEED);
            ((DcMotor) rightRearMotor.getHardwareDevice()).setPower(Constants.DEFAULT_SPEED);
        }

        if (robotStates.contains(RobotStates.ROTATE_CLOCKWISE)) {
            ((DcMotor) leftFrontMotor.getHardwareDevice()).setPower(Constants.DEFAULT_SPEED);
            ((DcMotor) leftRearMotor.getHardwareDevice()).setPower(-Constants.DEFAULT_SPEED);
            ((DcMotor) rightFrontMotor.getHardwareDevice()).setPower(Constants.DEFAULT_SPEED);
            ((DcMotor) rightRearMotor.getHardwareDevice()).setPower(-Constants.DEFAULT_SPEED);
        }

        if (robotStates.contains(RobotStates.ROTATE_COUNTERCLOCKWISE)) {
            ((DcMotor) leftFrontMotor.getHardwareDevice()).setPower(-Constants.DEFAULT_SPEED);
            ((DcMotor) leftRearMotor.getHardwareDevice()).setPower(Constants.DEFAULT_SPEED);
            ((DcMotor) rightFrontMotor.getHardwareDevice()).setPower(-Constants.DEFAULT_SPEED);
            ((DcMotor) rightRearMotor.getHardwareDevice()).setPower(Constants.DEFAULT_SPEED);
        }

        if (robotStates.contains(RobotStates.DRIVE_BRAKE)) {
            ((DcMotor) leftFrontMotor.getHardwareDevice()).setPower(0);
            ((DcMotor) leftFrontMotor.getHardwareDevice()).setPower(0);
            ((DcMotor) leftFrontMotor.getHardwareDevice()).setPower(0);
            ((DcMotor) leftFrontMotor.getHardwareDevice()).setPower(0);
        }
    }

    @Override
    public void stop() {
        ((DcMotor) leftFrontMotor.getHardwareDevice()).setPower(0);
        ((DcMotor) leftRearMotor.getHardwareDevice()).setPower(0);
        ((DcMotor) rightFrontMotor.getHardwareDevice()).setPower(0);
        ((DcMotor) rightRearMotor.getHardwareDevice()).setPower(0);
    }

    public int getEncoderAverage() {
        return (leftFrontMotor.getEncoderCount() + leftRearMotor.getEncoderCount()
                + rightFrontMotor.getEncoderCount() + rightRearMotor.getEncoderCount()) / 4; //Math.abs or nah
    }

    /**
     * Retrieves absolute value of encoder counts
     * Used for east and west movement where absolute value of average returns near 0
     *
     * @return average absolute value of all drive encoders
     */
    //TODO: Could be used to calc north and south too since math.abs of all = math.abs of some
    public int getAbsEncoderAverage() {
        return (Math.abs(leftFrontMotor.getEncoderCount()) + Math.abs(leftRearMotor.getEncoderCount())
                + Math.abs(rightFrontMotor.getEncoderCount()) + Math.abs(rightRearMotor.getEncoderCount())) / 4;
    }

    private void setDirections() {
        ((DcMotor) leftFrontMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.FORWARD);
        ((DcMotor) leftRearMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.FORWARD);
        ((DcMotor) rightFrontMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.REVERSE);
        ((DcMotor) rightRearMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setDriveMotorMode() {
        if (getGameMode().equals(GameMode.AUTO)) {
            getRobot().resetMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER,
                    (DcMotor) leftFrontMotor.getHardwareDevice(),
                    (DcMotor) leftRearMotor.getHardwareDevice(),
                    (DcMotor) rightFrontMotor.getHardwareDevice(),
                    (DcMotor) rightRearMotor.getHardwareDevice());
        } else if (getGameMode().equals(GameMode.TELEOP)) {
            getRobot().resetMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                    (DcMotor) leftFrontMotor.getHardwareDevice(),
                    (DcMotor) leftRearMotor.getHardwareDevice(),
                    (DcMotor) rightFrontMotor.getHardwareDevice(),
                    (DcMotor) rightRearMotor.getHardwareDevice());
        }
    }

    public void setDriveMotorMode(DcMotor.RunMode runMode) {
        getRobot().resetMotorsMode(runMode,
                (DcMotor) leftFrontMotor.getHardwareDevice(),
                (DcMotor) leftRearMotor.getHardwareDevice(),
                (DcMotor) rightFrontMotor.getHardwareDevice(),
                (DcMotor) rightRearMotor.getHardwareDevice());
    }
}
