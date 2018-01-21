package org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.hardware;

import org.firstinspires.ftc.teamcode.hardware.devices.RobotMotor;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PID;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PIDBuilder;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.consts.PIDConstants;

//swittch to using get input on average of all encoders
public class PIDDrive implements PID.PidInput {

    private PID drivePIDController = null;
    private RobotMotor leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor = null;

    public PIDDrive(PIDConstants pidConstants, RobotMotor leftFrontMotor, RobotMotor leftRearMotor,
                    RobotMotor rightFrontMotor, RobotMotor rightRearMotor) { //specify encoder or gyro
        PIDBuilder drivePIDBuilder = new PIDBuilder().setKP(pidConstants.getKP())
                .setKI(pidConstants.getKI())
                .setKD(pidConstants.getKD())
                .setKF(pidConstants.getKF())
                .setTOLERANCE(pidConstants.getTOLERANCE())
                .setSETTLING_TIME(pidConstants.getSETTLING_TIME())
                .setTARGET_RANGE(pidConstants.getTARGET_MIN_RANGE(), pidConstants.getTARGET_MAX_RANGE())
                .setTARGET(pidConstants.getTARGET())
                .setOUTPUT_RANGE(pidConstants.getMIN_OUTPUT(), pidConstants.getMAX_OUTPUT())
                .setTARGET(pidConstants.getTARGET())
                .setINVERTED(pidConstants.isINVERTED())
                .setABSOLUTE_SETPOINT(pidConstants.isABSOLUTE_SETPOINT())
                .setNO_OSCILLATION(pidConstants.isNO_OSCILLATION());

        this.leftFrontMotor = leftFrontMotor;
        this.leftRearMotor = leftRearMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.rightRearMotor = rightRearMotor;

        drivePIDController = new PID(drivePIDBuilder, this);
    }

    public boolean isOnTarget() {
        return drivePIDController.isOnTarget();
    }

    public double getOutput() {
        return getOutputPower();
    }

    public void reset() {
        drivePIDController.reset();
    }

    //Clean class to use getEncoderAvg from drivesystem
    @Override
    public double getInput(PID pid) {
        return (Math.abs(leftFrontMotor.getEncoderCount()) + Math.abs(leftRearMotor.getEncoderCount())
                + Math.abs(rightFrontMotor.getEncoderCount()) + Math.abs(rightRearMotor.getEncoderCount())) / 4;
    }

    private double getOutputPower() {
        return drivePIDController.getOutput();
    }
}
