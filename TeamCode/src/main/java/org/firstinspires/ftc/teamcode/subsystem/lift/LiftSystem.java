package org.firstinspires.ftc.teamcode.subsystem.lift;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;
import org.firstinspires.ftc.teamcode.hardware.controllers.RobotMotorController;
import org.firstinspires.ftc.teamcode.hardware.devices.RobotMotor;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.consts.CustomPIDConstants;
import org.firstinspires.ftc.teamcode.terminators.Status;
import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;

public class LiftSystem extends Subsystem {

    private RobotMotorController liftMotorController = null;
    private RobotMotor liftMotor = null;
    private DcMotor.RunMode liftMotorRunMode = DcMotor.RunMode.RUN_USING_ENCODER;

    private double liftSpeed = 0.5;
    private LiftLevel currentLiftLevel = LiftLevel.FIRST_FLOOR;

    //    private double TICKS_PER_REVOLUTION = 1440;
//    private double WHEEL_DIAMETER = 1;
    private double TICKS_PER_LEVEL = /* (TICKS_PER_REVOLUTION) * 6 / (WHEEL_DIAMETER * Math.PI); */ 2750;

    public LiftSystem(Robot robot, GameMode gameMode) {
        super(robot);
        setGameMode(gameMode);

        init();
    }

    @Override
    public void init() {
//        liftMotorController = getRobot().addMotorController(PrometheusConstants.LIFT_DRIVE_MOTOR_CONTROLLER);
        liftMotor = getRobot().addMotor(PrometheusConstants.LIFT_MOTOR);

        setDirection();
        setMotorMode();
    }

    @Override
    public void handle(ArrayList<RobotStates> robotStates) {
        if (robotStates.contains(RobotStates.RAISE_LIFT)) {
            liftMotor.setPower(liftSpeed);
        }

        if (robotStates.contains(RobotStates.DECREMENT_LIFT)) {
            liftMotor.setPower(-liftSpeed);
        }

        if (robotStates.contains(RobotStates.STOP_LIFT)) {
            liftMotor.setPower(0);
        }

        if (robotStates.contains(RobotStates.INCREMENT_LIFT)) {
//            incrementLift();
        }

        if (robotStates.contains(RobotStates.DECREMENT_LIFT)) {
//            decrementLift();
        }
    }

    @Override
    public void stop() {

    }

    private void setDirection() {
        ((DcMotor) liftMotor.getHardwareDevice()).setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setMotorMode() {
        ((DcMotor) liftMotor.getHardwareDevice()).setMode(liftMotorRunMode);
    }

//    private void incrementLift() {
//        if (!currentLiftLevel.equals(LiftLevel.FOURTH_FLOOR)) {
//            moveLift(1);
//            switch (currentLiftLevel) {
//                case FIRST_FLOOR:
//                    currentLiftLevel = LiftLevel.SECOND_FLOOR;
//                    break;
//                case SECOND_FLOOR:
//                    currentLiftLevel = LiftLevel.THIRD_FLOOR;
//                    break;
//                case THIRD_FLOOR:
//                    currentLiftLevel = LiftLevel.FOURTH_FLOOR;
//                    break;
//            }
//        }
//    }
//
//    private void decrementLift() {
//        if (!currentLiftLevel.equals(LiftLevel.FIRST_FLOOR)) {
//            moveLift(-1);
//            switch (currentLiftLevel) {
//                case SECOND_FLOOR:
//                    currentLiftLevel = LiftLevel.FIRST_FLOOR;
//                    break;
//                case THIRD_FLOOR:
//                    currentLiftLevel = LiftLevel.SECOND_FLOOR;
//                    break;
//                case FOURTH_FLOOR:
//                    currentLiftLevel = LiftLevel.THIRD_FLOOR;
//                    break;
//            }
//        }
//    }

    public void moveLift(double directionMultiplier, double inches) {
        double customDriveSpeed = liftSpeed;

        liftMotor.resetEncoderCount();
        liftMotor.setRunMode(liftMotorRunMode);

        double targetTicks = TICKS_PER_LEVEL / 6 * inches;

        liftMotor.setPower(customDriveSpeed);
        while (Math.abs(liftMotor.getEncoderCount()) < targetTicks && Status.isStopRequested()) {
            //Set speed to slow down as approaching target (0.5 inches away)
            double ticksRemaining = TICKS_PER_LEVEL - Math.abs(liftMotor.getEncoderCount());
            double inchesRemaining = ticksRemaining / TICKS_PER_LEVEL;

            customDriveSpeed = directionMultiplier * customDriveSpeed *
                    inchesRemaining * new CustomPIDConstants().setKP(0.6).getKP();
            liftMotor.setPower(customDriveSpeed);
        }

        liftMotor.setPower(0);
    }

    public void moveLift(double directionMultiplier, LiftLevel liftLevel) {
        double customDriveSpeed = liftSpeed;

        liftMotor.resetEncoderCount();
        liftMotor.setRunMode(liftMotorRunMode);

        double targetTicks = TICKS_PER_LEVEL;

        switch (liftLevel) {

            case SECOND_FLOOR:
                targetTicks = TICKS_PER_LEVEL * 1;
                break;
            case THIRD_FLOOR:
                targetTicks = TICKS_PER_LEVEL * 2;
                break;
            case FOURTH_FLOOR:
                targetTicks = TICKS_PER_LEVEL * 3;
                break;
        }

        liftMotor.setPower(customDriveSpeed);
        while (Math.abs(liftMotor.getEncoderCount()) < targetTicks && Status.isStopRequested()) {
            //Set speed to slow down as approaching target (0.5 inches away)
            double ticksRemaining = TICKS_PER_LEVEL - Math.abs(liftMotor.getEncoderCount());
            double inchesRemaining = ticksRemaining / TICKS_PER_LEVEL;

            customDriveSpeed = directionMultiplier * customDriveSpeed *
                    inchesRemaining * new CustomPIDConstants().setKP(0.6).getKP();
            liftMotor.setPower(customDriveSpeed);
        }

        liftMotor.setPower(0);
    }
}
