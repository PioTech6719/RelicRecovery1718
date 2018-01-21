package org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.movement.encoder;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;
import org.firstinspires.ftc.teamcode.subsystem.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PIDComplexity;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.consts.CustomPIDConstants;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.consts.DrivePIDConstants;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.consts.GyroPIDConstants;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.hardware.PIDDrive;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.hardware.PIDGyro;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.movement.MovementController;
import org.firstinspires.ftc.teamcode.subsystem.sensors.gyro.GyroSensorSystem;
import org.firstinspires.ftc.teamcode.terminators.Status;
import org.firstinspires.ftc.teamcode.terminators.Terminator;
import org.firstinspires.ftc.teamcode.utils.PioTimer;

public class EncoderGyroMovementController implements MovementController {

    private Robot robot = null; //Purpose??
    private DriveSystem driveSystem = null; //Since this needed to drive mb remove Robot
    private DcMotor leftFront, leftRear, rightFront, rightRear = null; //Don't use obtain from drivesystem

    //PID Systems
    private PIDDrive pidDrive = null;
    private PIDGyro pidGyro = null;

    private double DRIVE_GEAR_REDUCTION = 1.0;
    private double TICKS_PER_REVOLUTION = 1120;
    private double WHEEL_DIAMETER = 4;
    private double TICKS_PER_INCH = (TICKS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER * Math.PI);

    private double defaultDriveSpeed = 1.0;
    private DcMotor.RunMode motorRunMode = null;

    private ElapsedTime timer = new ElapsedTime();

    //TODO: Later use motor pairs and allow motor pair ot only have 1 motor do math accordingly for encoder etc
    public EncoderGyroMovementController(@NonNull Robot robot, DriveSystem driveSystem,
                                         double DRIVE_GEAR_REDUCTION,
                                         double defaultDriveSpeed,
                                         DcMotor.RunMode motorRunMode) {
        this.robot = robot;
        this.driveSystem = driveSystem;
        this.leftFront = ((DcMotor) driveSystem.getLeftFrontMotor().getHardwareDevice());
        this.leftRear = ((DcMotor) driveSystem.getLeftRearMotor().getHardwareDevice());
        this.rightFront = ((DcMotor) driveSystem.getRightFrontMotor().getHardwareDevice());
        this.rightRear = ((DcMotor) driveSystem.getRightRearMotor().getHardwareDevice());
        this.DRIVE_GEAR_REDUCTION = DRIVE_GEAR_REDUCTION;
        this.defaultDriveSpeed = defaultDriveSpeed;
        this.motorRunMode = motorRunMode;
    }

    public double getDefaultDriveSpeed() {
        return defaultDriveSpeed;
    }

    public void setDefaultDriveSpeed(double defaultDriveSpeed) {
        this.defaultDriveSpeed = defaultDriveSpeed;
    }

    @Override
    public void move(Directions direction, double inches, PioTimer timer, PIDComplexity pidComplexity) {
        robot.resetMotorsMode(motorRunMode, leftFront, leftRear, rightFront, rightRear);

        if (pidComplexity.equals(PIDComplexity.NONE)) {

            simpleMove(direction, inches, defaultDriveSpeed, timer);

        } else if (pidComplexity.equals(PIDComplexity.LOW)) {

            simplePIDMove(direction, inches, defaultDriveSpeed, timer);

        } else if (pidComplexity.equals(PIDComplexity.MEDIUM)) {

            //NOT SUPPORTED

        } else if (pidComplexity.equals(PIDComplexity.HIGH)) {
            complexPIDMove(direction, inches, defaultDriveSpeed, timer);
        }
    }

    @Override
    public void move(Directions direction, double inches,
                     double customDriveSpeed, PioTimer timer, PIDComplexity pidComplexity) {
        robot.resetMotorsMode(motorRunMode, leftFront, leftRear, rightFront, rightRear);

        switch (pidComplexity) {

            case NONE:
                simpleMove(direction, inches, customDriveSpeed, timer);
                break;
            case LOW:
                simplePIDMove(direction, inches, customDriveSpeed, timer);
                break;
            case MEDIUM:
                break;
            case HIGH:
                complexPIDMove(direction, inches, customDriveSpeed, timer);
                break;

        }
    }

    @Override
    public void move(Directions direction, PioTimer pioTimer, Terminator terminator) {
        pioTimer.start();
        while (!terminator.shouldTerminate() && !pioTimer.isFinished() && !Status.isStopRequested()) {
            switch (direction) {
                case NORTH:
                    leftFront.setPower(defaultDriveSpeed);
                    leftRear.setPower(defaultDriveSpeed);
                    rightFront.setPower(defaultDriveSpeed);
                    rightRear.setPower(defaultDriveSpeed);
                    break;
                case SOUTH:
                    leftFront.setPower(-defaultDriveSpeed);
                    leftRear.setPower(-defaultDriveSpeed);
                    rightFront.setPower(-defaultDriveSpeed);
                    rightRear.setPower(-defaultDriveSpeed);
                    break;
                case EAST:
                    leftFront.setPower(defaultDriveSpeed);
                    leftRear.setPower(-defaultDriveSpeed);
                    rightFront.setPower(-defaultDriveSpeed);
                    rightRear.setPower(defaultDriveSpeed);
                    break;
                case WEST:
                    leftFront.setPower(-defaultDriveSpeed);
                    leftRear.setPower(defaultDriveSpeed);
                    rightFront.setPower(defaultDriveSpeed);
                    rightRear.setPower(-defaultDriveSpeed);
                    break;
            }
        }
    }

    @Override
    public void move(Directions direction, double customDriveSpeed, PioTimer pioTimer,
                     Terminator terminator) {
        pioTimer.start();
        while (!terminator.shouldTerminate() && !pioTimer.isFinished() && !Status.isStopRequested()) {
            switch (direction) {
                case NORTH:
                    leftFront.setPower(customDriveSpeed);
                    leftRear.setPower(customDriveSpeed);
                    rightFront.setPower(customDriveSpeed);
                    rightRear.setPower(customDriveSpeed);
                    break;
                case SOUTH:
                    leftFront.setPower(-customDriveSpeed);
                    leftRear.setPower(-customDriveSpeed);
                    rightFront.setPower(-customDriveSpeed);
                    rightRear.setPower(-customDriveSpeed);
                    break;
                case EAST:
                    leftFront.setPower(customDriveSpeed);
                    leftRear.setPower(-customDriveSpeed);
                    rightFront.setPower(-customDriveSpeed);
                    rightRear.setPower(customDriveSpeed);
                    break;
                case WEST:
                    leftFront.setPower(-customDriveSpeed);
                    leftRear.setPower(customDriveSpeed);
                    rightFront.setPower(customDriveSpeed);
                    rightRear.setPower(-customDriveSpeed);
                    break;
            }
        }

    }

    public void simpleMove(Directions direction, double inches, double driveSpeed, PioTimer timer) {
        double customDriveSpeed = driveSpeed;
        double targetTicks = inches * TICKS_PER_INCH;

        switch (direction) {
            case NORTH:
                leftFront.setPower(customDriveSpeed);
                leftRear.setPower(customDriveSpeed);
                rightFront.setPower(customDriveSpeed);
                rightRear.setPower(customDriveSpeed);
                break;
            case SOUTH:
                leftFront.setPower(-customDriveSpeed);
                leftRear.setPower(-customDriveSpeed);
                rightFront.setPower(-customDriveSpeed);
                rightRear.setPower(-customDriveSpeed);
                break;
            case EAST:
                leftFront.setPower(customDriveSpeed);
                leftRear.setPower(-customDriveSpeed);
                rightFront.setPower(-customDriveSpeed);
                rightRear.setPower(customDriveSpeed);
                break;
            case WEST:
                leftFront.setPower(-customDriveSpeed);
                leftRear.setPower(customDriveSpeed);
                rightFront.setPower(customDriveSpeed);
                rightRear.setPower(-customDriveSpeed);
                break;
        }

        if (Robot.getOpMode() instanceof LinearOpMode) {
            while (((LinearOpMode) Robot.getOpMode()).opModeIsActive() &&
                    Math.abs(driveSystem.getAbsEncoderAverage()) < targetTicks) {
                //Set speed to slow down as approaching target (0.5 inches away)
                double ticksRemaining = targetTicks - Math.abs(driveSystem.getEncoderAverage());
                double inchesRemaining = ticksRemaining / TICKS_PER_INCH;

                customDriveSpeed = customDriveSpeed * inchesRemaining * new CustomPIDConstants().setKP(0.6).getKP();

                //TODO: Replace with a move method given directions which is cleaner than this
                switch (direction) {
                    case NORTH:
                        leftFront.setPower(customDriveSpeed);
                        leftRear.setPower(customDriveSpeed);
                        rightFront.setPower(customDriveSpeed);
                        rightRear.setPower(customDriveSpeed);
                        break;
                    case SOUTH:
                        leftFront.setPower(-customDriveSpeed);
                        leftRear.setPower(-customDriveSpeed);
                        rightFront.setPower(-customDriveSpeed);
                        rightRear.setPower(-customDriveSpeed);
                        break;
                    case EAST:
                        leftFront.setPower(customDriveSpeed);
                        leftRear.setPower(-customDriveSpeed);
                        rightFront.setPower(-customDriveSpeed);
                        rightRear.setPower(customDriveSpeed);
                        break;
                    case WEST:
                        leftFront.setPower(-customDriveSpeed);
                        leftRear.setPower(customDriveSpeed);
                        rightFront.setPower(customDriveSpeed);
                        rightRear.setPower(-customDriveSpeed);
                        break;
                }
            }
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public void simplePIDMove(Directions direction, double inches, double driveSpeed, PioTimer timer) {
        double target = TICKS_PER_INCH * inches;
        int clicksRemaining;
        double inchesRemaining, power, averageClicks;

        timer.start();
        do {
            averageClicks = driveSystem.getAbsEncoderAverage(); /*- this works but that doesnt driveSystem.getAbsEncoderAverage() *///prev: Math.abs(getEncoderAvg())
            clicksRemaining = (int) (target - averageClicks);
            inchesRemaining = clicksRemaining / TICKS_PER_INCH;

            power = driveSpeed * inchesRemaining * new CustomPIDConstants().setKP(0.6).getKP();
            power = Range.clip(power, -1, 1);

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
                    leftFront.setPower(power);
                    leftRear.setPower(-power);
                    rightFront.setPower(-power);
                    rightRear.setPower(power);
                    break;
                case WEST:
                    leftFront.setPower(-power);
                    leftRear.setPower(power);
                    rightFront.setPower(power);
                    rightRear.setPower(-power);
                    break;
            }
            //chg 0.5 to tolerance var
        } while (inchesRemaining > 0.5 && !timer.isFinished() && !Status.isStopRequested());

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public void complexPIDMove(Directions direction, double inches, double driveSpeed, PioTimer timer) {
        DrivePIDConstants drivePIDConstants = new DrivePIDConstants().setTARGET(inches * TICKS_PER_INCH);
        pidDrive = new PIDDrive(drivePIDConstants, driveSystem.getLeftFrontMotor(), driveSystem.getLeftRearMotor(),
                driveSystem.getRightFrontMotor(), driveSystem.getRightRearMotor());

        double power;

        timer.start();
        while (!pidDrive.isOnTarget() && !timer.isFinished()) {
            power = pidDrive.getOutput(); //Use power to increment drive speed or multiplier

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
                    leftFront.setPower(power);
                    leftRear.setPower(-power);
                    rightFront.setPower(-power);
                    rightRear.setPower(power);
                    break;
                case WEST:
                    leftFront.setPower(-power);
                    leftRear.setPower(power);
                    rightFront.setPower(power);
                    rightRear.setPower(-power);
                    break;
            }
        }

        leftFront.setPower(0); //Have method with sets to all in robot given list setPwoer(power, DcMotors...)
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    @Override
    public void rotate(Directions direction, double angle, PioTimer timer, PIDComplexity pidComplexity) {
        robot.resetMotorsMode(motorRunMode, leftFront, leftRear, rightFront, rightRear);

        if (pidComplexity.equals(PIDComplexity.NONE)) {

            rotateTo(angle);

        } else if (pidComplexity.equals(PIDComplexity.LOW)) {

            simplePIDRotate(direction, angle, timer);

        } else if (pidComplexity.equals(PIDComplexity.MEDIUM)) {

            //NOT SUPPORTED

        } else if (pidComplexity.equals(PIDComplexity.HIGH)) {

            complexPIDRotate(direction, angle, defaultDriveSpeed, timer);
        }
    }

    @Override
    public void rotate(Directions direction, double angle, double driveSpeed, PioTimer timer, PIDComplexity pidComplexity) {

    }

    public void rotateTo(double desiredHeading) { //mbjust use and chg constant.tolerance for now
        robot.resetMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER, leftFront, leftRear,
                rightFront, rightRear);

        GyroSensorSystem gyroSensorSystem = ((GyroSensorSystem) robot.getSubsystem(GyroSensorSystem.class));
        double currHeading = gyroSensorSystem.getPureHeading();

        //ConvertToPosHeading.
        double posCurrHeading = (currHeading >= 0) ? currHeading : currHeading + 360;

        if (desiredHeading - posCurrHeading > 180) {
            //Rotate CCW
            //mb chg curr to posHeading
            while (Math.abs(currHeading - desiredHeading) > PrometheusConstants.TOLERANCE && !Status.isStopRequested()) {
                leftFront.setPower(0.375);
                rightFront.setPower(-0.375);
                leftRear.setPower(0.375);
                rightRear.setPower(-0.375);
                currHeading = gyroSensorSystem.getPureHeading();
            }
        } else if (desiredHeading - posCurrHeading < 180) {
            //Rotate CW
            while (Math.abs(currHeading - desiredHeading) > PrometheusConstants.TOLERANCE && !Status.isStopRequested()) {
                leftFront.setPower(-0.375);
                rightFront.setPower(0.375);
                leftRear.setPower(-0.375);
                rightRear.setPower(0.375);
                currHeading = gyroSensorSystem.getPureHeading();
            }
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    /**
     * Rotates the robot by calculating the difference between the desired angle and the current
     * heading.
     * <p>
     * Does not rotate to a set angle which is in:
     * {@link EncoderGyroMovementController#rotateTo(double, double, PioTimer, PIDComplexity)}
     * <p>
     * Current limitation is robot can only do angles < 180 due to the heading of the gyro resetting direction at that point
     * Fix: Use custom getHeading var to set back to 0-360
     *
     * @param direction of rotation
     * @param angle     to rotate
     * @param timer     to stop motion TODO: to be replaced by terminator
     */
    private void simplePIDRotate(Directions direction, double angle, PioTimer timer) {
        GyroSensorSystem gyroSensorSystem = ((GyroSensorSystem) robot.getSubsystem(GyroSensorSystem.class));

        //Check if gyroSensorSystem is present on robot.
        if (gyroSensorSystem != null && angle < 180) {
            gyroSensorSystem.resetAngles();

            //Calculate initial error
            double error = angle - Math.abs(gyroSensorSystem.getHeading());

            //Init power variable (not sure whether to set to drive speed or not and use correct value to rotate)
            //Also not sure to start rotating at start then update or do only in while
            double power;

            //Should timer start here or in loop but then isFinished won't work
            timer.start();

            //Run while error from angle is larger than 3 (tolerance) or timer not completed
            while (Math.abs(error) > 5 && !timer.isFinished() && !Status.isStopRequested()) {
                //Set power based on error
                power = error * new CustomPIDConstants().setKP(0.03).getKP();

                //Set power to rotate
                switch (direction) {
                    case ROTATE_CW:
                        leftFront.setPower(-power);
                        leftRear.setPower(-power);
                        rightFront.setPower(power);
                        rightRear.setPower(power);
                        break;
                    case ROTATE_CCW:
                        leftFront.setPower(power);
                        leftRear.setPower(power);
                        rightFront.setPower(-power);
                        rightRear.setPower(-power);
                        break;
                }

                //Update error value
                error = angle - gyroSensorSystem.getHeading();
            } //mb use do while and use angles remaning too - mediumPID

            //Stop all motors upon reaching target
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
    }

    private void complexPIDRotate(Directions direction, double angle, double driveSpeed, PioTimer timer) {
        GyroSensorSystem gyroSensorSystem = ((GyroSensorSystem) robot.getSubsystem(GyroSensorSystem.class));

        if (gyroSensorSystem != null) {
            //Since this is a simple rotate and not rotateTo(), must reset angles
            gyroSensorSystem.resetAngles();

            //Initialize PID constants and assign to PID controller
            GyroPIDConstants gyroPIDConstants = new GyroPIDConstants().setTARGET(angle);
            pidGyro = new PIDGyro(gyroPIDConstants, driveSystem, gyroSensorSystem, direction, 0.5);

            timer.start();

            //Run while not on PID target or timer not completed
            while (!pidGyro.isOnTarget() && !timer.isFinished() && !Status.isStopRequested()) {

                //Retrieve correction value from PID
                double correctionVal = pidGyro.getOutput();

                //Apply correction value to default power depending on
                switch (direction) {
                    case ROTATE_CW:
                        leftFront.setPower(-driveSpeed - correctionVal); //or drivespeed - correctionVal? this would do straight driving but idk abt rot.
                        leftRear.setPower(-driveSpeed - correctionVal);
                        rightFront.setPower(driveSpeed + correctionVal);
                        rightRear.setPower(driveSpeed + correctionVal);
                        break;
                    case ROTATE_CCW:
                        leftFront.setPower(driveSpeed + correctionVal);
                        leftRear.setPower(driveSpeed + correctionVal);
                        rightFront.setPower(-driveSpeed - correctionVal);
                        rightRear.setPower(-driveSpeed - correctionVal);
                        break;
                }
            }

            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
    }

    /**
     * Rotates to a certain absolute heading from 0-360 by calculating absolute value difference
     * between desired and current heading.
     * Limitation: Currently may not use the fastest path to achieve target.
     * (Ex: may go CW 330 when CCW 30 is faster)
     *
     * @param targetAngle   angle desired from 0-360 from initial starting point
     * @param timer         to determine termination time
     * @param pidComplexity to determine comp
     */
    @Override
    public void rotateTo(double targetAngle, PioTimer timer, PIDComplexity pidComplexity) {
        //Double instantiation in this and rotate()
        GyroSensorSystem gyroSensorSystem = ((GyroSensorSystem) robot.getSubsystem(GyroSensorSystem.class));

        //Default value to prevent NPE
        Directions rotationDirection = Directions.ROTATE_CW;

        if (targetAngle > gyroSensorSystem.getAbsoluteHeading()) {
            rotationDirection = Directions.ROTATE_CW;
        } else if (targetAngle < gyroSensorSystem.getAbsoluteHeading()) {
            rotationDirection = Directions.ROTATE_CCW;
        }

        rotate(rotationDirection,
                Math.abs(targetAngle) - Math.abs(gyroSensorSystem.getAbsoluteHeading()),
                timer,
                pidComplexity);
    }

    @Override
    public void rotateTo(double targetAngle, double driveSpeed, PioTimer timer, PIDComplexity pidComplexity) {
        //Double instantiation in this and rotate()
        GyroSensorSystem gyroSensorSystem = ((GyroSensorSystem) robot.getSubsystem(GyroSensorSystem.class));

        //Default value to prevent NPE
        Directions rotationDirection = Directions.ROTATE_CW;

        if (targetAngle > gyroSensorSystem.getAbsoluteHeading()) {
            rotationDirection = Directions.ROTATE_CW;
        } else if (targetAngle < gyroSensorSystem.getAbsoluteHeading()) {
            rotationDirection = Directions.ROTATE_CCW;
        }

        rotate(rotationDirection,
                Math.abs(targetAngle) - Math.abs(gyroSensorSystem.getAbsoluteHeading()),
                timer,
                pidComplexity);
    }


    /**
     * TODO: make all these centralized and for specific robots so that once speicifcs are clicked
     * the same moves can be played. for example dpad up no matter joystick moves up
     */
}
