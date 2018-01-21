package org.firstinspires.ftc.teamcode.opmodes.auto.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.gamepads.configs.PrometheusConfigTwo;
import org.firstinspires.ftc.teamcode.hardware.robots.Prometheus;
import org.firstinspires.ftc.teamcode.match.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutoOp;
import org.firstinspires.ftc.teamcode.subsystem.jewel.JewelSensorSystem;
import org.firstinspires.ftc.teamcode.subsystem.sensors.ColorSensorSystem;

@Autonomous(name = "Sensor Test - Color Sensor", group = "test")
public class ColorSensorTest extends BaseAutoOp {

    private Alliance alliance = Alliance.BLUE;
    private Prometheus prometheus = null;
    private PrometheusConfigTwo gamepadConfig = null;

    @Override
    public void initializeClass() {
        prometheus = new Prometheus(this);

        setRobot(prometheus);
        setGamepadConfig(null);
        setAlliance(alliance);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        ColorSensorSystem colorSensorSystem =
                ((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class))
                        .getJewelColorSensorSystem();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Color Sensor Hue", colorSensorSystem.getHue());
            telemetry.addData("Color Sensor Red", colorSensorSystem.getRed());
            telemetry.addData("Color Sensor Green", colorSensorSystem.getGreen());
            telemetry.addData("Color Sensor Blue", colorSensorSystem.getBlue());
            telemetry.update();
        }
    }
}
