package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class Prometheus {

    static final double DRIVE_GEAR_REDUCTION = 0.5; //Geared up 1:2
    public HardwareMap hwMap = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor liftMotor = null;
    public Servo ballPush = null;
    public Servo arm1 = null;
    public Servo arm2 = null;
    public ColorSensor colorSensor = null;
    public BNO055IMU imu = null;
    public double ticks = 1120;
    public double wheelDiameter = 4;
    public double inchesPerTick = (ticks * DRIVE_GEAR_REDUCTION) / (wheelDiameter * Math.PI);
    public LiftSetting currentLiftSetting = LiftSetting.LOW;
    private ElapsedTime time = new ElapsedTime();

    //seperate into initauto and inittele bc of gyro
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        frontLeft = hwMap.get(DcMotor.class, "front_left");
        frontRight = hwMap.get(DcMotor.class, "front_right");
        backLeft = hwMap.get(DcMotor.class, "back_left");
        backRight = hwMap.get(DcMotor.class, "back_right");
        liftMotor = hwMap.get(DcMotor.class, "liftMotor");
        ballPush = (Servo) hwMap.get("ball_push");
        arm1 = hwMap.get(Servo.class, "arm1");
        arm2 = hwMap.get(Servo.class, "arm2");
        colorSensor = hwMap.get(ColorSensor.class, "Color_Sensor");
        imu = (BNO055IMU) hwMap.get("imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; //
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        ballArmReset();
        openGlyphArms();
    }

    public void initTele(HardwareMap hwMap) {
        this.hwMap = hwMap;

        frontLeft = hwMap.get(DcMotor.class, "front_left");
        frontRight = hwMap.get(DcMotor.class, "front_right");
        backLeft = hwMap.get(DcMotor.class, "back_left");
        backRight = hwMap.get(DcMotor.class, "back_right");
        liftMotor = hwMap.get(DcMotor.class, "liftMotor");
        ballPush = (Servo) hwMap.get("ball_push");
        arm1 = hwMap.get(Servo.class, "arm1");
        arm2 = hwMap.get(Servo.class, "arm2");
        colorSensor = hwMap.get(ColorSensor.class, "Color_Sensor");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        ballArmReset();
        openGlyphArms();
    }

    public int getColor() {
        if (colorSensor.red() > colorSensor.blue()) {
            return Color.RED;
        } else {
            return Color.BLUE;
        }
    }

    /**
     * Certain power to left motors and right motors mainly for 6wd
     * Multiple wya to do this
     * Left and right y-axis or arcade drive
     * @param leftPower
     * @param rightPower
     */
    public void driveLeftRight(double leftPower, double rightPower) {
        if (Math.abs(leftPower) > Constants.GAMEPAD_THRESHOLD) {
            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);
        }

        if (Math.abs(rightPower) > Constants.GAMEPAD_THRESHOLD) {
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);
        }
    }

    //Drive Scheme 1

    public void driveXY(double xVal, double yVal) {
        if (Math.abs(yVal) > Constants.GAMEPAD_THRESHOLD && Math.abs(xVal) < Constants.GAMEPAD_THRESHOLD) { // North or South
            frontLeft.setPower(yVal);
            frontRight.setPower(yVal);
            backLeft.setPower(yVal);
            backRight.setPower(yVal);
        } else if (Math.abs(xVal) > Constants.GAMEPAD_THRESHOLD && Math.abs(yVal) < Constants.GAMEPAD_THRESHOLD) { // East or West
            frontLeft.setPower(xVal);
            frontRight.setPower(-xVal);
            backLeft.setPower(-xVal);
            backRight.setPower(xVal);
        }
    }

    public void rotateXY(double val) {
        if (Math.abs(val) > Constants.GAMEPAD_THRESHOLD) {
            frontLeft.setPower(val);
            frontRight.setPower(-val);
            backLeft.setPower(val);
            backRight.setPower(-val);
        }
    }

    public void move(LinearOpMode opMode, Directions direction, int distance) {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (direction == Directions.NORTH) {
            frontLeft.setTargetPosition(frontLeft.getTargetPosition() + (int) (distance * inchesPerTick));
            frontRight.setTargetPosition(frontRight.getTargetPosition() + (int) (distance * inchesPerTick));
            backLeft.setTargetPosition(backLeft.getTargetPosition() + (int) (distance * inchesPerTick));
            backRight.setTargetPosition(backRight.getTargetPosition() + (int) (distance * inchesPerTick));
        } else if (direction == Directions.SOUTH) {
            frontLeft.setTargetPosition(frontLeft.getTargetPosition() - (int) (distance * inchesPerTick));
            frontRight.setTargetPosition(frontRight.getTargetPosition() - (int) (distance * inchesPerTick));
            backLeft.setTargetPosition(backLeft.getTargetPosition() - (int) (distance * inchesPerTick));
            backRight.setTargetPosition(backRight.getTargetPosition() - (int) (distance * inchesPerTick));
        } else if (direction == Directions.EAST) {
            frontLeft.setTargetPosition(frontLeft.getTargetPosition() + (int) (distance * inchesPerTick));
            frontRight.setTargetPosition(frontRight.getTargetPosition() - (int) (distance * inchesPerTick));
            backLeft.setTargetPosition(backLeft.getTargetPosition() - (int) (distance * inchesPerTick));
            backRight.setTargetPosition(backRight.getTargetPosition() + (int) (distance * inchesPerTick));
        } else if (direction == Directions.WEST) {
            frontLeft.setTargetPosition(frontLeft.getTargetPosition() - (int) (distance * inchesPerTick));
            frontRight.setTargetPosition(frontRight.getTargetPosition() + (int) (distance * inchesPerTick));
            backLeft.setTargetPosition(backLeft.getTargetPosition() + (int) (distance * inchesPerTick));
            backRight.setTargetPosition(backRight.getTargetPosition() - (int) (distance * inchesPerTick));
        } else if (direction == Directions.ROTATE_CW) {
            frontLeft.setTargetPosition(frontLeft.getTargetPosition() + (int) (distance * inchesPerTick));
            frontRight.setTargetPosition(frontRight.getTargetPosition() - (int) (distance * inchesPerTick));
            backLeft.setTargetPosition(backLeft.getTargetPosition() + (int) (distance * inchesPerTick));
            backRight.setTargetPosition(backRight.getTargetPosition() - (int) (distance * inchesPerTick));
        } else if (direction == Directions.ROTATE_CCW) {
            frontLeft.setTargetPosition(frontLeft.getTargetPosition() - (int) (distance * inchesPerTick));
            frontRight.setTargetPosition(frontRight.getTargetPosition() + (int) (distance * inchesPerTick));
            backLeft.setTargetPosition(backLeft.getTargetPosition() - (int) (distance * inchesPerTick));
            backRight.setTargetPosition(backRight.getTargetPosition() + (int) (distance * inchesPerTick));
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (direction == Directions.NORTH) {
            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);
        } else if (direction == Directions.SOUTH) {
            frontLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(-0.5);
        }

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void move(double lspeed, double rspeed, LinearOpMode opMode, Directions direction, int distance, double timeout) {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int leftTarget = (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition()) / 2 +
                (int) (distance * inchesPerTick);
        int rightTarget = (frontRight.getCurrentPosition() + backRight.getCurrentPosition()) / 2 +
                (int) (distance * inchesPerTick);
        time.reset();

        while ((time.seconds() < timeout) && (Math.abs(frontLeft.getCurrentPosition() + backLeft.getCurrentPosition()) / 2 < leftTarget && Math.abs(frontRight.getCurrentPosition() + backRight.getCurrentPosition()) / 2 < rightTarget)) {
            double rem = (Math.abs(frontLeft.getCurrentPosition()) + Math.abs(backLeft.getCurrentPosition()) + Math.abs(frontRight.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition())) / 4;

            double nlSpeed = 0;
            double nrSpeed = 0;

            double R = time.seconds();
            if (R < 4) {
                double ramp = R / 4;
                nlSpeed = lspeed * ramp;
                nrSpeed = rspeed * ramp;
            } else if (rem > (1000)) {
                nlSpeed = lspeed;
                nrSpeed = rspeed;
            } else if (rem > (200) && (0.5 * .2) > 0.1 && (0.5 * .2) > .1) {
                nlSpeed = lspeed * .2;
                nrSpeed = rspeed * .2;
            }

            frontLeft.setPower(nlSpeed);
            frontRight.setPower(nrSpeed);
            backLeft.setPower(nlSpeed);
            backRight.setPower(nrSpeed);
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void setLiftSystem(LiftSetting desiredLiftSetting) {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setTargetPosition(liftMotor.getTargetPosition() +
                (int) (desiredLiftSetting.heightVal - currentLiftSetting.heightVal * inchesPerTick));

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(desiredLiftSetting.getHeightVal() > currentLiftSetting.getHeightVal() ? 1 : -1);

        while (liftMotor.isBusy()) {}

        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Why specify dir when angle magnitude determines direction
    public void imuRotate(double angle, Directions direction, double TOLERANCE) {
        double currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double desiredHeading = currHeading + angle;

        if (direction == Directions.ROTATE_CW) {
            while (Math.abs(currHeading - desiredHeading) > TOLERANCE) {
                frontLeft.setPower(0.3);
                frontRight.setPower(-0.3);
                backLeft.setPower(0.3);
                backRight.setPower(-0.3);
                currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        } else if (direction == Directions.ROTATE_CCW) {
            while (Math.abs(currHeading - desiredHeading) > TOLERANCE) {
                frontLeft.setPower(-0.3);
                frontRight.setPower(0.3);
                backLeft.setPower(-0.3);
                backRight.setPower(0.3);
                currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        }
    }

    public void imuRotateTo(double desiredHeading, Directions direction, double TOLERANCE) {
        double currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if (direction.equals(Directions.ROTATE_CCW)) {
            while (Math.abs(currHeading - desiredHeading) > TOLERANCE) {
                frontLeft.setPower(-0.3);
                frontRight.setPower(0.3);
                backLeft.setPower(-0.3);
                backRight.setPower(0.3);
                currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        } else if (direction.equals(Directions.ROTATE_CW)) {
            while (Math.abs(currHeading - desiredHeading) > TOLERANCE) {
                frontLeft.setPower(0.3);
                frontRight.setPower(-0.3);
                backLeft.setPower(0.3);
                backRight.setPower(-0.3);
                currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        }
    }

    public void imuRotateTo(double desiredHeading, double TOLERANCE) {
        //mb convert desired to positive value too or throw error if neg.
        double currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //ConvertToPosHeading.
        double posCurrHeading = (currHeading >= 0) ? currHeading : currHeading + 360;

        if (desiredHeading - posCurrHeading > 180 || desiredHeading - posCurrHeading < 0) {
            //Rotate CW
            //mb chg curr to posHeading
            while (Math.abs(currHeading - desiredHeading) > TOLERANCE) {
                frontLeft.setPower(0.3);
                frontRight.setPower(-0.3);
                backLeft.setPower(0.3);
                backRight.setPower(-0.3);
                currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        } else if (desiredHeading - posCurrHeading < 180) {
            //Rotate CCW
            while (Math.abs(currHeading - desiredHeading) > TOLERANCE) {
                frontLeft.setPower(-0.3);
                frontRight.setPower(0.3);
                backLeft.setPower(-0.3);
                backRight.setPower(0.3);
                currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        }
    }

    public void ballArmReset() {
        ballPush.setPosition(0.5);
    }

    public void ballArmUp() {
        ballPush.setPosition(1.0);
    }

    public void ballArmDown() {
        ballPush.setPosition(0.0);
    }

    public void openGlyphArms() {
        arm1.setPosition(-30);
        arm2.setPosition(60);
    }

    public void closeGlyphArms() {
        arm1.setPosition(30);
        arm2.setPosition(0);
    }

    public enum LiftSetting {
        LOW(0), MID(10), HIGH(20);

        private int heightVal;

        LiftSetting(int heightVal) {
            this.heightVal = heightVal;
        }

        public int getHeightVal() {
            return heightVal;
        }
    }
}