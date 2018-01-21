package org.firstinspires.ftc.teamcode.opmodes.auto.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.robots.Prometheus;
import org.firstinspires.ftc.teamcode.match.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutoOp;
import org.firstinspires.ftc.teamcode.subsystem.sensors.LightSensorSystem;

@Autonomous(name = "Sensor Test - Light Sensor", group = "test")
public class LightSensorTest extends BaseAutoOp {

    private Alliance alliance = Alliance.BLUE;
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
        super.runOpMode();

        waitForStart();

        while (opModeIsActive()) {
            LightSensorSystem lightSensorSystem =
                    ((LightSensorSystem) prometheus.getSubsystem(LightSensorSystem.class));

            telemetry.addData("Light Value: ", lightSensorSystem.getLight());
            telemetry.addData("Raw Light Value: ", lightSensorSystem.getRawLight());
            telemetry.update();
        }
    }
}
