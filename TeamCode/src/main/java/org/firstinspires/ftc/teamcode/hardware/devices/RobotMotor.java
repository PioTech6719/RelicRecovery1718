package org.firstinspires.ftc.teamcode.hardware.devices;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotHardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.controllers.RobotMotorController;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PID;

public class RobotMotor extends RobotHardwareDevice implements PID.PidInput {

    private RobotMotorController motorController = null;

    public RobotMotor(@NonNull DcMotor motor) {
        super(motor, "");
    }

    public RobotMotor(@NonNull DcMotor motor, @NonNull String name) {
        super(motor, name);
        //Maybe use .getDeviceName from DcMotor class instead?
    }

    public RobotMotor(@NonNull RobotMotorController motorController, @NonNull DcMotor motor, @NonNull String name) {
        super(motor, name);
        //Maybe use .getDeviceName from DcMotor class instead?
        this.motorController = motorController;
    }

    public RobotMotorController getMotorController() {
        return motorController;
    }

    public void setMotorController(@NonNull RobotMotorController motorController) {
        this.motorController = motorController;
    }

    //TODO: Implement
    public boolean isStalling() {
        return false;
    }

    @Override
    public double getInput(PID pid) {
        return getEncoderCount(); //TODO:ticks or inches?
    }

    public int getEncoderCount() {
        return ((DcMotor) getHardwareDevice()).getCurrentPosition();
    }

    public void resetEncoderCount() {
        DcMotor.RunMode previousRunMode = ((DcMotor) getHardwareDevice()).getMode();
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (Robot.getOpMode() instanceof LinearOpMode) {
            ((LinearOpMode) Robot.getOpMode()).idle();
        }

        setRunMode(previousRunMode);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        ((DcMotor) getHardwareDevice()).setDirection(direction);
    }

    public void setPower(double power) {
        ((DcMotor) getHardwareDevice()).setPower(power);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        ((DcMotor) getHardwareDevice()).setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        ((DcMotor) getHardwareDevice()).setZeroPowerBehavior(zeroPowerBehavior);
    }
}
