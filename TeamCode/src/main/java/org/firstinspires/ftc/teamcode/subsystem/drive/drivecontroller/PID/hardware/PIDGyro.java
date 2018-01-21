package org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.hardware;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.devices.RobotMotor;
import org.firstinspires.ftc.teamcode.subsystem.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PID;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PIDBuilder;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.consts.PIDConstants;
import org.firstinspires.ftc.teamcode.subsystem.sensors.gyro.GyroSensorSystem;

public class PIDGyro implements PID.PidInput {

    private PID drivePIDController = null;
    private RobotMotor leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor = null;
    private GyroSensorSystem gyroSensorSystem;
    private Directions direction;
    private double defaultPower;

    public PIDGyro(PIDConstants pidConstants, DriveSystem driveSystem, GyroSensorSystem gyroSensorSystem,
                   Directions direction, double defaultPower) { //specify encoder or gyro
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

        this.leftFrontMotor = driveSystem.getLeftFrontMotor();
        this.leftRearMotor = driveSystem.getLeftRearMotor();
        this.rightFrontMotor = driveSystem.getRightFrontMotor();
        this.rightRearMotor = driveSystem.getRightRearMotor();
        this.gyroSensorSystem = gyroSensorSystem;
        this.direction = direction;
        this.defaultPower = defaultPower;

        drivePIDController = new PID(drivePIDBuilder, this);
    }

    public boolean isOnTarget() {
        return drivePIDController.isOnTarget();
    }

    /**
     * Returns the output of the PID loop
     *
     * @return correction value to add or subtract original power from depending on direction of rot.
     */
    public double getOutput() {
        return getOutputPower();
    }

    public void reset() {
        drivePIDController.reset();
    }

    @Override
    public double getInput(PID pid) {
        return gyroSensorSystem.getHeading();
    }

    private double getOutputPower() {
        return drivePIDController.getOutput();
    }
}
