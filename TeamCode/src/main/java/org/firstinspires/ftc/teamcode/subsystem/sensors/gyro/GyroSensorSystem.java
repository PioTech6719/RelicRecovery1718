package org.firstinspires.ftc.teamcode.subsystem.sensors.gyro;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
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
    private double[] angles;
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

    public double adjustAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    /**
     * Return heading in 0-180 (going CCW) & 0-(-180) (going CW) format
     *
     * @return
     */
    public double getHeading() {
        updateValues();
        return angles[0] - zeroPosition;
    }

    /**
     * Returns heading in 0-360 clockwise format from 0-180 (going CCW) & 0-(-180) (going CW) format
     * Useful for rotateTo() functions
     *
     * @return heading in 0-360 format
     */
    public double getAbsoluteHeading() {
        updateValues();

        double currentAbsoluteAngle = angles[0];

        if (angles[0] >= 0 && angles[0] < 180) {
            currentAbsoluteAngle = angles[0] + 180;
        } else if (angles[0] < 0 && angles[0] >= -180) {
            currentAbsoluteAngle = Math.abs(angles[0]);
        }

        return currentAbsoluteAngle - initialZeroPosition;
    }

    public double getPureHeading() {
        updateValues();
        return angles[0];
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
        Quaternion quatAngles = gyroSensor.getQuaternionOrientation();
        double w = quatAngles.w;
        double x = quatAngles.x;
        double y = quatAngles.y;
        double z = quatAngles.z;

        double roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 180.0 / Math.PI;
        double pitch = Math.asin(2 * (w * y - x * z)) * 180.0 / Math.PI;
        double yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 180.0 / Math.PI;

        angles = new double[]{yaw, pitch, roll};
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
                .addData("Heading : ", new Func<Double>() {
                    @Override
                    public Double value() {
                        return angles[0];
                    }
                })
                .addData("Absolute Heading : ", new Func<Double>() {
                    @Override
                    public Double value() {
                        return getAbsoluteHeading();
                    }
                })
                .addData("Adjusted Pure Heading : ", new Func<Double>() {
                    @Override
                    public Double value() {
                        return adjustAngle(angles[0]);
                    }
                }).addData("Real Heading: ", new Func<Float>() {
            @Override
            public Float value() {
                return gyroSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        })

                .addData("Roll : ", new Func<Double>() {
                    @Override
                    public Double value() {
                        return angles[2];
                    }
                })
                .addData("Pitch : ", new Func<Double>() {
                    @Override
                    public Double value() {
                        return angles[1];
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
}