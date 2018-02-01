package org.firstinspires.ftc.teamcode.hardware.robots.old;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class Pionizer {

    public HardwareMap hwMap = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor liftMotor = null;
    public Servo ballPush = null;

    public Servo upperLeftServo = null;
    public Servo upperRightServo = null;
    public Servo lowerLeftServo = null;
    public Servo lowerRightServo = null;

    public ColorSensor colorSensor = null;
    public BNO055IMU imu = null;

    private MoveProvider moveProvider = null;
    private JewelProvider jewelProvider = null;

    private ElapsedTime time = new ElapsedTime();

    public MoveProvider getMoveProvider() {
        return moveProvider;
    }

    public JewelProvider getJewelProvider() {
        return jewelProvider;
    }

    public void initAuto(HardwareMap hwMap) {
        this.hwMap = hwMap;

        frontLeft = hwMap.get(DcMotor.class, PrometheusConstants.LEFT_FRONT_DRIVE_MOTOR);
        frontRight = hwMap.get(DcMotor.class, PrometheusConstants.RIGHT_FRONT_DRIVE_MOTOR);
        backLeft = hwMap.get(DcMotor.class, PrometheusConstants.LEFT_REAR_DRIVE_MOTOR);
        backRight = hwMap.get(DcMotor.class, PrometheusConstants.RIGHT_REAR_DRIVE_MOTOR);
        liftMotor = hwMap.get(DcMotor.class, PrometheusConstants.LIFT_MOTOR);
        ballPush = (Servo) hwMap.get(PrometheusConstants.JEWEL_SERVO);
        upperLeftServo = hwMap.get(Servo.class, PrometheusConstants.UPPER_LEFT_SERVO);
        upperRightServo = hwMap.get(Servo.class, PrometheusConstants.UPPER_RIGHT_SERVO);
        lowerLeftServo = hwMap.get(Servo.class, PrometheusConstants.LOWER_LEFT_SERVO);
        lowerRightServo = hwMap.get(Servo.class, PrometheusConstants.LOWER_RIGHT_SERVO);

        colorSensor = hwMap.get(ColorSensor.class, PrometheusConstants.JEWEL_COLOR_DISTANCE_SENSOR);
        imu = (BNO055IMU) hwMap.get(PrometheusConstants.GYRO_SENSOR);
        initGyro();

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initTele(HardwareMap hwMap) {
        this.hwMap = hwMap;

        frontLeft = hwMap.get(DcMotor.class, PrometheusConstants.LEFT_FRONT_DRIVE_MOTOR);
        frontRight = hwMap.get(DcMotor.class, PrometheusConstants.RIGHT_FRONT_DRIVE_MOTOR);
        backLeft = hwMap.get(DcMotor.class, PrometheusConstants.LEFT_REAR_DRIVE_MOTOR);
        backRight = hwMap.get(DcMotor.class, PrometheusConstants.RIGHT_REAR_DRIVE_MOTOR);
        liftMotor = hwMap.get(DcMotor.class, PrometheusConstants.LIFT_MOTOR);
        ballPush = (Servo) hwMap.get(PrometheusConstants.JEWEL_SERVO);
        upperLeftServo = hwMap.get(Servo.class, PrometheusConstants.UPPER_LEFT_SERVO);
        upperRightServo = hwMap.get(Servo.class, PrometheusConstants.UPPER_RIGHT_SERVO);
        lowerLeftServo = hwMap.get(Servo.class, PrometheusConstants.LOWER_LEFT_SERVO);
        lowerRightServo = hwMap.get(Servo.class, PrometheusConstants.LOWER_RIGHT_SERVO);

        colorSensor = hwMap.get(ColorSensor.class, PrometheusConstants.JEWEL_COLOR_DISTANCE_SENSOR);
        imu = (BNO055IMU) hwMap.get(PrometheusConstants.GYRO_SENSOR);
        initGyro();

        moveProvider = new MoveProvider(frontLeft, backLeft, frontRight, backRight, imu);
        jewelProvider = new JewelProvider(colorSensor);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initServos() {
        upperLeftServo.setPosition(0.06);
        upperRightServo.setPosition(0.44);
        lowerLeftServo.setPosition(0.13);
        lowerRightServo.setPosition(0.78);
        ballPush.setPosition(0.3);
    }

    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
    }

    public void move(double leftStickX, double leftStickY, double rightStickX, double rightStickY) {
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

        if (Math.abs(rightStickY) > Constants.GAMEPAD_THRESHOLD) {
            rightY = rightStickY;
        }

        frontLeft.setPower(leftY + leftX + rightX);
        frontRight.setPower(leftY - leftX - rightX);
        backLeft.setPower(leftY + leftX - rightX);
        backRight.setPower(leftY - leftX + rightX);
    }

//    public void move(double lspeed, double rspeed, LinearOpMode opMode, Directions direction, int distance, double timeout) {
//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        opMode.idle();
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        int leftTarget = (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition()) / 2 +
//                (int) (distance * ticksPerInch);
//        int rightTarget = (frontRight.getCurrentPosition() + backRight.getCurrentPosition()) / 2 +
//                (int) (distance * ticksPerInch);
//        time.reset();
//
//        while ((time.seconds() < timeout) && (Math.abs(frontLeft.getCurrentPosition() + backLeft.getCurrentPosition()) / 2 < leftTarget && Math.abs(frontRight.getCurrentPosition() + backRight.getCurrentPosition()) / 2 < rightTarget)) {
//            double rem = (Math.abs(frontLeft.getCurrentPosition()) + Math.abs(backLeft.getCurrentPosition()) + Math.abs(frontRight.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition())) / 4;
//
//            double nlSpeed = 0;
//            double nrSpeed = 0;
//
//            double R = time.seconds();
//            if (R < 4) {
//                double ramp = R / 4;
//                nlSpeed = lspeed * ramp;
//                nrSpeed = rspeed * ramp;
//            } else if (rem > (1000)) {
//                nlSpeed = lspeed;
//                nrSpeed = rspeed;
//            } else if (rem > (200) && (0.5 * .2) > 0.1 && (0.5 * .2) > .1) {
//                nlSpeed = lspeed * .2;
//                nrSpeed = rspeed * .2;
//            }
//
//            frontLeft.setPower(nlSpeed);
//            frontRight.setPower(nrSpeed);
//            backLeft.setPower(nlSpeed);
//            backRight.setPower(nrSpeed);
//        }
//
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        try {
//            Thread.sleep(250);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//    }

    public void imuRotateTo(double desiredHeading, double TOLERANCE) {
        //mb convert desired to positive value too or throw error if neg.
        double currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //ConvertToPosHeading.
        double posCurrHeading = (currHeading >= 0) ? currHeading : currHeading + 360;

        if (desiredHeading - posCurrHeading > 180) {
            //Rotate CCW
            //mb chg curr to posHeading
            while (Math.abs(currHeading - desiredHeading) > TOLERANCE) {
                frontLeft.setPower(0.3);
                frontRight.setPower(-0.3);
                backLeft.setPower(0.3);
                backRight.setPower(-0.3);
                currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        } else if (desiredHeading - posCurrHeading < 180) {
            //Rotate CW
            while (Math.abs(currHeading - desiredHeading) > TOLERANCE) {
                frontLeft.setPower(-0.3);
                frontRight.setPower(0.3);
                backLeft.setPower(-0.3);
                backRight.setPower(0.3);
                currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void ballArmReset() {
        ballPush.setPosition(0.3);
    }

    public void ballArmUp() {
        ballPush.setPosition(0.3);
    }

    //TODO: Don't smash arm against wall. go slowly towards target
    public void ballArmDown() {
        ballPush.setPosition(0.86);
    }

    public void openUpperGlyphArms() {
        upperLeftServo.setPosition(0.06);
        upperRightServo.setPosition(0.44);
    }

    public void closeUpperGlyphArms() {
        upperLeftServo.setPosition(0.34);
        upperRightServo.setPosition(0.20);
    }

    public void openLowerGlyphArms() {
        lowerLeftServo.setPosition(0.13);
        lowerRightServo.setPosition(0.78);
    }

    public void closeLowerGlyphArms() {
        lowerLeftServo.setPosition(0.02);
        lowerRightServo.setPosition(0.90);
    }
}