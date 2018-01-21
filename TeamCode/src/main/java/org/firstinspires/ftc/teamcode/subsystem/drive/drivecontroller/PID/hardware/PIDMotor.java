package org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.hardware;

import org.firstinspires.ftc.teamcode.hardware.devices.RobotMotor;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PID;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PIDBuilder;

//TODO: move to hardware
//Unused now bc PIDDrive simply combines all motors for a drivetrain drive(no longer uses pidmotor)
//later on build pid based off each motor
public class PIDMotor { //organize these logcally mb under drivecontroller put movement classes and use this things get leftfront power method (once made). have bool to turn off pid or inclue in method parameter

    private PID motorPIDControl = null;
    private RobotMotor robotMotor = null;

    public PIDMotor(RobotMotor robotMotor, PIDBuilder pidBuilder) {
        motorPIDControl = new PID(pidBuilder, robotMotor);
        this.robotMotor = robotMotor;

        if (pidBuilder.getMAX_OUTPUT() != 0 && pidBuilder.getMIN_OUTPUT() != 0) {
            motorPIDControl.setOutputRange(pidBuilder.getMIN_OUTPUT(), pidBuilder.getMAX_OUTPUT());
        }

        if (pidBuilder.getTARGET_MAX_RANGE() != 0 && pidBuilder.getTARGET_MIN_RANGE() != 0) {
            motorPIDControl.setTargetRange(pidBuilder.getTARGET_MIN_RANGE(), pidBuilder.getTARGET_MAX_RANGE());
        }

        motorPIDControl.setInverted(pidBuilder.isINVERTED());
        motorPIDControl.setAbsoluteSetPoint(pidBuilder.isABSOLUTE_SETPOINT());
        motorPIDControl.setNoOscillation(pidBuilder.isNO_OSCILLATION());
    }

    public RobotMotor getRobotMotor() {
        return robotMotor;
    }

    public void setTarget(double encoderCounts, double tolerance) {
        motorPIDControl.setTarget(encoderCounts);
        motorPIDControl.setTargetTolerance(tolerance);
    }

    public boolean isOnTarget() {
        return motorPIDControl.isOnTarget();
    }

    public double getOutputPower() {
        return motorPIDControl.getOutput();
    }

    public void reset() {
        motorPIDControl.reset();
    }
}
