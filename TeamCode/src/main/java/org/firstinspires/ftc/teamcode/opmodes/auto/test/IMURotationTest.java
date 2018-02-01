package org.firstinspires.ftc.teamcode.opmodes.auto.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;
import org.firstinspires.ftc.teamcode.utils.Constants;

import java.util.Locale;

@Autonomous(name = "Sensor Test - IMU", group = "test")
//@Disabled
//TODO: Rewrite in BaseAutoOp w/ Prometheus and take the align method from pionizer to prom
//TODO: Also update imu at end of move not beginning
public class IMURotationTest extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private BNO055IMU imu = null;
    private double defaultLinearSpeed = 0.8;
    private Orientation angles = null;
    private Acceleration gravity;

    @Override
    public void runOpMode() throws InterruptedException {
        connectDevices();
        initGyro();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        logTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            //Linear + Rotational Movement
            if (gamepad1.dpad_up) { //North

                frontLeft.setPower(defaultLinearSpeed);
                frontRight.setPower(defaultLinearSpeed);
                backLeft.setPower(defaultLinearSpeed);
                backRight.setPower(defaultLinearSpeed);

            } else if (gamepad1.dpad_down) {

                frontLeft.setPower(-defaultLinearSpeed);
                frontRight.setPower(-defaultLinearSpeed);
                backLeft.setPower(-defaultLinearSpeed);
                backRight.setPower(-defaultLinearSpeed);

            } else if (gamepad1.dpad_left) {

                frontLeft.setPower(-defaultLinearSpeed);
                frontRight.setPower(defaultLinearSpeed);
                backLeft.setPower(defaultLinearSpeed);
                backRight.setPower(-defaultLinearSpeed);

            } else if (gamepad1.dpad_right) {

                frontLeft.setPower(defaultLinearSpeed);
                frontRight.setPower(-defaultLinearSpeed);
                backLeft.setPower(-defaultLinearSpeed);
                backRight.setPower(defaultLinearSpeed);

            } else if (gamepad1.left_trigger > Constants.GAMEPAD_THRESHOLD) { //Rotate Left

                frontLeft.setPower(-0.75);
                frontRight.setPower(0.75);
                backLeft.setPower(-0.75);
                backRight.setPower(0.75);

            } else if (gamepad1.right_trigger > Constants.GAMEPAD_THRESHOLD) { //Rotate Right

                frontLeft.setPower(0.75);
                frontRight.setPower(-0.75);
                backLeft.setPower(0.75);
                backRight.setPower(-0.75);

            } else {

                frontLeft.setPower(0.0);
                frontRight.setPower(0.0);
                backLeft.setPower(0.0);
                backRight.setPower(0.0);

            }

            if (gamepad2.a) {
                double currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                //ConvertToPosHeading.
                double posCurrHeading = (currHeading >= 0) ? currHeading : currHeading + 360;

                if (90 - posCurrHeading > 180) {
                    //Rotate CCW
                    //mb chg curr to posHeading
                    while (Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - 90) > 5 && opModeIsActive()) {
                        frontLeft.setPower(0.3);
                        frontRight.setPower(-0.3);
                        backLeft.setPower(0.3);
                        backRight.setPower(-0.3);
                    }
                } else if (90 - posCurrHeading < 180) {
                    //Rotate CW
                    while (Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - 90) > 5 && opModeIsActive()) {
                        frontLeft.setPower(-0.3);
                        frontRight.setPower(0.3);
                        backLeft.setPower(-0.3);
                        backRight.setPower(0.3);
                    }
                }

                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }

            telemetry.update();
        }
    }

    private void connectDevices() {
        imu = (BNO055IMU) hardwareMap.get(PrometheusConstants.GYRO_SENSOR);
        frontLeft = (DcMotor) hardwareMap.get(PrometheusConstants.LEFT_FRONT_DRIVE_MOTOR);
        frontRight = (DcMotor) hardwareMap.get(PrometheusConstants.RIGHT_FRONT_DRIVE_MOTOR);
        backLeft = (DcMotor) hardwareMap.get(PrometheusConstants.LEFT_REAR_DRIVE_MOTOR);
        backRight = (DcMotor) hardwareMap.get(PrometheusConstants.RIGHT_REAR_DRIVE_MOTOR);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
    }

    private void logTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
