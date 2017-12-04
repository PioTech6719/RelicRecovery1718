package org.firstinspires.ftc.teamcode.opmodes.tele.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Prometheus;

@TeleOp(name = "Sensor - Color Sensor", group = "test")
public class REVColorTest extends OpMode {

    private Prometheus robot = new Prometheus();

    @Override
    public void init() {
        robot.initTele(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Color Red : ", robot.colorSensor.red());
        telemetry.addData("Color Blue : ", robot.colorSensor.blue());
        telemetry.addData("Color Green : ", robot.colorSensor.green());
        telemetry.addData("Color Alpha : ", robot.colorSensor.alpha());
        telemetry.addData("Color ARGB : ", robot.colorSensor.argb());
        telemetry.update();
    }
}
