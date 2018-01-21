package org.firstinspires.ftc.teamcode.subsystem.sensors.gyro;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;
import java.util.Locale;

public class GyroSensorSystem extends Subsystem {

    private BNO055IMU gyroSensor = null;
    private Orientation angles;
    private Acceleration gravity;

    private double zeroPosition = 0;
    private double initialZeroPosition = 0;

    private double prevError = 0;
    private double integral = 0;

    private ElapsedTime runTime;

    public GyroSensorSystem(Robot robot, GameMode gameMode) {
        super(robot);
        setGameMode(gameMode);

        init();
    }

    @Override
    public void init() {
        gyroSensor = Robot.getOpMode().hardwareMap.get(BNO055IMU.class, PrometheusConstants.GYRO_SENSOR);
        runTime = new ElapsedTime();
        calibrateGyro();

        resetAngles();
        initialZeroPosition = zeroPosition;
    }

    @Override
    public void handle(ArrayList<RobotStates> robotStates) {

    }

    @Override
    public void stop() {
        gyroSensor.close();
    }

    private void calibrateGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new NaiveAccelerationIntegrator();

        gyroSensor.initialize(parameters);

        gyroSensor.startAccelerationIntegration(new Position(), new Velocity(), 100);
    }

    /**
     * Return heading in 0-180 (going CCW) & 0-(-180) (going CW) format
     *
     * @return
     */
    public double getHeading() {
        updateValues();
        return angles.firstAngle - zeroPosition;
    }

    /**
     * Returns heading in 0-360 clockwise format from 0-180 (going CCW) & 0-(-180) (going CW) format
     * Useful for rotateTo() functions
     *
     * @return heading in 0-360 format
     */
    public double getAbsoluteHeading() {
        updateValues();

        double currentAbsoluteAngle = angles.firstAngle;

        if (angles.firstAngle >= 0 && angles.firstAngle < 180) {
            currentAbsoluteAngle = angles.firstAngle + 180;
        } else if (angles.firstAngle < 0 && angles.firstAngle >= -180) {
            currentAbsoluteAngle = Math.abs(angles.firstAngle);
        }

        return currentAbsoluteAngle - initialZeroPosition;
    }

    public double getPureHeading() {
        updateValues();
        return angles.firstAngle;
    }

    public double getRate() {
        updateValues();
        return gyroSensor.getAngularVelocity().zRotationRate;
    }

    public Acceleration getAcceleration() {
        updateValues();
        return gyroSensor.getAcceleration();
    }

    public Velocity getVelocity() {
        updateValues();
        return gyroSensor.getVelocity();
    }

    public Position getPosition() {
        updateValues();
        return gyroSensor.getPosition();
    }

    private void updateValues() {
        angles = gyroSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = gyroSensor.getGravity();
    }

    public void resetAngles() {
        updateValues();
        zeroPosition = getHeading();
    }

    //move to drive system
//    public double getAlignmentCorrection(double angle) {
//        if (prevError == 0) {
//            runTime.reset();
//        }
//
//        final double heading = getHeading();
//        double error = angle - heading;
//
//        integral = integral + error * kP() * runTime.milliseconds();
//        double derivative = (error - prevError)/runTime.milliseconds();
//        double correction = error * kP() + derivative * kD() + integral * kI();
//
//        //correction = Range.clip(correction, 0, 0.3);
//
//        prevError = error;
//        runTime.reset();
//        return correction;
//    }

    public void addToTelemetry() {
        updateValues();

        Robot.getOpMode().telemetry.addLine()
                .addData("Status : ", new Func<String>() {
                    @Override
                    public String value() {
                        return gyroSensor.getSystemStatus().toShortString();
                    }
                })
                .addData("Calibration Status : ", new Func<String>() {
                    @Override
                    public String value() {
                        return gyroSensor.getCalibrationStatus().toString();
                    }
                });

        Robot.getOpMode().telemetry.addLine()
                .addData("Heading : ", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("Roll : ", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("Pitch : ", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        Robot.getOpMode().telemetry.addLine()
                .addData("Gravity : ", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("Magnetic Strength : ", new Func<String>() {
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
