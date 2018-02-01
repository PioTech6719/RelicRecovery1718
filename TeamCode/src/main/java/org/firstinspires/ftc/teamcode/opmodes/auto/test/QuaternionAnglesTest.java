package org.firstinspires.ftc.teamcode.opmodes.auto.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.robots.Prometheus;
import org.firstinspires.ftc.teamcode.match.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutoOp;
import org.firstinspires.ftc.teamcode.subsystem.sensors.gyro.GyroSensorSystem;

@Autonomous(name = "Sensor Test - Qua", group = "test")
public class QuaternionAnglesTest extends BaseAutoOp {

    private Alliance alliance = Alliance.RED;
    private Prometheus prometheus;

    @Override
    public void initializeClass() {
        prometheus = new Prometheus(this);

        setRobot(prometheus);
        setGamepadConfig(null);
        setAlliance(Alliance.BLUE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();

        super.runOpMode();
        GyroSensorSystem gyroSensorSystem = (GyroSensorSystem) prometheus.getSubsystem(GyroSensorSystem.class);

        telemetry.addData("Status: ", "Ready to Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.clearAll();
            gyroSensorSystem.addToTelemetry();
            sleep(2500);
            telemetry.update();
        }
    }
}