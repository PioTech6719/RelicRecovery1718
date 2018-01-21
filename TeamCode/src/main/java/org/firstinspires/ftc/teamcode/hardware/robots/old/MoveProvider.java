package org.firstinspires.ftc.teamcode.hardware.robots.old;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PID;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PIDBuilder;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class MoveProvider {

    private final double DRIVE_GEAR_REDUCTION = 2.0; //Geared up 1:2
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private BNO055IMU imu;
    private double driveSpeed = 0.9;
    private double TICKS = 1120;
    private double wheelDiameter = 4;
    private double ticksPerInch = (TICKS * DRIVE_GEAR_REDUCTION) / (wheelDiameter * Math.PI);

    private PID pid;
    private boolean pidInit = false;

    public MoveProvider(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear, BNO055IMU imu) {
        this.leftFront = leftFront;
        this.leftRear = leftRear;
        this.rightFront = rightFront;
        this.rightRear = rightRear;
        this.imu = imu;
    }

    public void setDriveSpeed(double driveSpeed) {
        this.driveSpeed = driveSpeed;
    }

    //Mb switch to RUN USING ENCODER
    public void moveRunToPosition(LinearOpMode opMode, Directions direction, int distance) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (direction == Directions.NORTH) {
            leftFront.setTargetPosition(leftFront.getCurrentPosition() + (int) (distance * ticksPerInch));
            rightFront.setTargetPosition(rightFront.getCurrentPosition() + (int) (distance * ticksPerInch));
            leftRear.setTargetPosition(leftRear.getCurrentPosition() + (int) (distance * ticksPerInch));
            rightRear.setTargetPosition(rightRear.getCurrentPosition() + (int) (distance * ticksPerInch));
        } else if (direction == Directions.SOUTH) {
            leftFront.setTargetPosition(leftFront.getCurrentPosition() - (int) (distance * ticksPerInch));
            rightFront.setTargetPosition(rightFront.getCurrentPosition() - (int) (distance * ticksPerInch));
            leftRear.setTargetPosition(leftRear.getCurrentPosition() - (int) (distance * ticksPerInch));
            rightRear.setTargetPosition(rightRear.getCurrentPosition() - (int) (distance * ticksPerInch));
        } else if (direction == Directions.EAST) {
            leftFront.setTargetPosition(leftFront.getCurrentPosition() + (int) (distance * ticksPerInch));
            rightFront.setTargetPosition(rightFront.getCurrentPosition() - (int) (distance * ticksPerInch));
            leftRear.setTargetPosition(leftRear.getCurrentPosition() - (int) (distance * ticksPerInch));
            rightRear.setTargetPosition(rightRear.getCurrentPosition() + (int) (distance * ticksPerInch));
        } else if (direction == Directions.WEST) {
            leftFront.setTargetPosition(leftFront.getCurrentPosition() - (int) (distance * ticksPerInch));
            rightFront.setTargetPosition(rightFront.getCurrentPosition() + (int) (distance * ticksPerInch));
            leftRear.setTargetPosition(leftRear.getCurrentPosition() + (int) (distance * ticksPerInch));
            rightRear.setTargetPosition(rightRear.getCurrentPosition() - (int) (distance * ticksPerInch));
        } else if (direction == Directions.ROTATE_CW) {
            leftFront.setTargetPosition(leftFront.getCurrentPosition() + (int) (distance * ticksPerInch));
            rightFront.setTargetPosition(rightFront.getCurrentPosition() - (int) (distance * ticksPerInch));
            leftRear.setTargetPosition(leftRear.getCurrentPosition() + (int) (distance * ticksPerInch));
            rightRear.setTargetPosition(rightRear.getCurrentPosition() - (int) (distance * ticksPerInch));
        } else if (direction == Directions.ROTATE_CCW) {
            leftFront.setTargetPosition(leftFront.getCurrentPosition() - (int) (distance * ticksPerInch));
            rightFront.setTargetPosition(rightFront.getCurrentPosition() + (int) (distance * ticksPerInch));
            leftRear.setTargetPosition(leftRear.getCurrentPosition() - (int) (distance * ticksPerInch));
            rightRear.setTargetPosition(rightRear.getCurrentPosition() + (int) (distance * ticksPerInch));
        }

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double customSpeed = driveSpeed;

        if (direction == Directions.NORTH) {
            leftFront.setPower(customSpeed);
            rightFront.setPower(customSpeed);
            leftRear.setPower(customSpeed);
            rightRear.setPower(customSpeed);
        } else if (direction == Directions.SOUTH) {
            leftFront.setPower(-customSpeed);
            rightFront.setPower(-customSpeed);
            leftRear.setPower(-customSpeed);
            rightRear.setPower(-customSpeed);
        } else if (direction == Directions.EAST) {
            leftFront.setPower(customSpeed);
            rightFront.setPower(-customSpeed);
            leftRear.setPower(-customSpeed);
            rightRear.setPower(customSpeed);
        } else if (direction == Directions.WEST) {
            leftFront.setPower(-customSpeed);
            rightFront.setPower(customSpeed);
            leftRear.setPower(customSpeed);
            rightRear.setPower(-customSpeed);
        }

        while (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {
            //Calculate distance left and slow down
            double averageEncoderCounts = -(leftFront.getCurrentPosition() + leftRear.getCurrentPosition()
                    + rightFront.getCurrentPosition() + rightRear.getCurrentPosition());
            double inchesRemaining = (distance * ticksPerInch) - averageEncoderCounts * ticksPerInch;
            customSpeed = driveSpeed * inchesRemaining * 0.6;
            customSpeed = Range.clip(customSpeed, -1, 1);
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveSimplePID(LinearOpMode opMode, DcMotor.RunMode runMode, Directions direction, double distanceInches) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftRear.setMode(runMode);
        rightRear.setMode(runMode);

        double target = ticksPerInch * distanceInches;
        int clicksRemaining;
        double inchesRemaining, power;

        double kp = 0.6;
        do {
            clicksRemaining = (int) (target - Math.abs((leftFront.getCurrentPosition() + rightFront.getCurrentPosition()
                    + leftRear.getCurrentPosition() + rightRear.getCurrentPosition()) / 4));
            inchesRemaining = clicksRemaining / ticksPerInch;

            switch (direction) {
                case NORTH:
                    power = 1 * 0.9 * inchesRemaining * kp; //*KP
                    power = Range.clip(power, -1, 1);

                    leftFront.setPower(power);
                    rightFront.setPower(power);
                    leftRear.setPower(power);
                    rightRear.setPower(power);

                    break;
                case SOUTH:
                    power = 1 * 0.9 * inchesRemaining * kp; //*KP
                    power = Range.clip(power, -1, 1);

                    leftFront.setPower(-power);
                    rightFront.setPower(-power);
                    leftRear.setPower(-power);
                    rightRear.setPower(-power);

                    break;
                case EAST:
                    power = 1 * 0.9 * inchesRemaining * kp; //*KP
                    power = Range.clip(power, -1, 1);

                    leftFront.setPower(power);
                    rightFront.setPower(-power);
                    leftRear.setPower(-power);
                    rightRear.setPower(power);

                    break;
                case WEST:
                    power = 1 * 0.9 * inchesRemaining * kp; //*KP
                    power = Range.clip(power, -1, 1);

                    leftFront.setPower(-power);
                    rightFront.setPower(power);
                    leftRear.setPower(power);
                    rightRear.setPower(-power);

                    break;
            }
        } while (inchesRemaining > 0.5);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void moveFullPID(LinearOpMode opMode, DcMotor.RunMode runMode, Directions direction, double distanceInches, double encoderTolerance) {
        OldPID pid = new OldPID(DrivePIDConstants.KP, DrivePIDConstants.KI, DrivePIDConstants.KD, DrivePIDConstants.DELAY, DrivePIDConstants.INTEGRAL_RANGE, DrivePIDConstants.OUTPUT_RANGE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftRear.setMode(runMode);
        rightRear.setMode(runMode);

        double inputEncoderCounts = -(leftFront.getCurrentPosition() + leftRear.getCurrentPosition() + rightFront.getCurrentPosition() + rightRear.getCurrentPosition());
        double target = ticksPerInch * distanceInches;
        pid.setTarget(target);

        while (opMode.opModeIsActive() && Math.abs(target - inputEncoderCounts) > encoderTolerance) {
            //Update power here or end of loop
            inputEncoderCounts = -(leftFront.getCurrentPosition() + leftRear.getCurrentPosition() + rightFront.getCurrentPosition() + rightRear.getCurrentPosition());
            double power = pid.update(inputEncoderCounts);

            switch (direction) {

                case NORTH:
                    leftFront.setPower(power);
                    leftRear.setPower(power);
                    rightFront.setPower(power);
                    rightRear.setPower(power);
                    break;
                case SOUTH:
                    leftFront.setPower(-power);
                    leftRear.setPower(-power);
                    rightFront.setPower(-power);
                    rightRear.setPower(-power);
                    break;
                case EAST:
                    leftFront.setPower(-power);
                    leftRear.setPower(power);
                    rightFront.setPower(power);
                    rightRear.setPower(-power);
                    break;
                case WEST:
                    leftFront.setPower(power);
                    leftRear.setPower(-power);
                    rightFront.setPower(-power);
                    rightRear.setPower(power);
                    break;
            }
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        pid.reset();
    }

    //Why specify dir when angle magnitude determines direction
    public void imuRotate(double angle, Directions direction, double TOLERANCE) {
        double currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double desiredHeading = currHeading + angle;

        if (direction.equals(Directions.ROTATE_CW)) {
            while (Math.abs(currHeading - desiredHeading) > TOLERANCE) {
                leftFront.setPower(0.4);
                rightFront.setPower(-0.4);
                leftRear.setPower(0.4);
                rightRear.setPower(-0.4);
                currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        } else if (direction.equals(Directions.ROTATE_CCW)) {
            while (Math.abs(currHeading - desiredHeading) > TOLERANCE) {
                leftFront.setPower(-0.4);
                rightFront.setPower(0.4);
                leftRear.setPower(-0.4);
                rightRear.setPower(0.4);
                currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        }

        //Always zero power after move
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    private void initPID() {
        if (!pidInit) {
            //Initialize PID
            PIDBuilder pidBuilder = new PIDBuilder()
                    .setKP(0.06)
                    .setKI(0.015)
                    .setKD(0.04)
                    .setKF(0.03)
                    .setTARGET(0)
                    .setTOLERANCE(4)
                    .setOUTPUT_RANGE(-0.5, 0.5)
                    .setSETTLING_TIME(1);

            pid = new PID(pidBuilder, new PID.PidInput() {
                @Override
                public double getInput(PID pid) {
                    return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
            });

            pidInit = true;
        }
    }

    public void driveStraight(boolean reset, double leftStickX, double leftStickY, double rightStickX) {
        //Setup PID
        initPID();

        if (reset) {
            pid.reset();
        }

        //Obtain Correction Value for PID
        double correctionVal = pid.getOutput();

        double leftX = 0, leftY = 0, rightX = 0, rightY = 0;

        if (Math.abs(leftStickX) > Constants.GAMEPAD_THRESHOLD) {
            leftX = leftStickX;
        }

        if (Math.abs(leftStickY) > Constants.GAMEPAD_THRESHOLD) {
            leftY = leftStickY;
        }

        if (Math.abs(rightStickX) > Constants.GAMEPAD_THRESHOLD) {
            rightX = rightStickX;
        }

        double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if (heading > 0) { //Left of 0 so more power to left motors
            leftFront.setPower(leftY + leftX + rightX + correctionVal);
            rightFront.setPower(leftY - leftX - rightX - correctionVal);
            leftRear.setPower(leftY + leftX - rightX + correctionVal);
            rightRear.setPower(leftY - leftX + rightX - correctionVal);
        } else if (heading < 0) { //Right of 0 so more power to right motors
            leftFront.setPower(leftY + leftX + rightX - correctionVal);
            rightFront.setPower(leftY - leftX - rightX + correctionVal);
            leftRear.setPower(leftY + leftX - rightX - correctionVal);
            rightRear.setPower(leftY - leftX + rightX + correctionVal);
        } else {
            leftFront.setPower(leftY + leftX + rightX);
            rightFront.setPower(leftY - leftX - rightX);
            leftRear.setPower(leftY + leftX - rightX);
            rightRear.setPower(leftY - leftX + rightX);
        }
    }
}
