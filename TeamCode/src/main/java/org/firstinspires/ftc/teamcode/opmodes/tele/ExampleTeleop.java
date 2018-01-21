package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.gamepads.configs.PrometheusConfigTwo;
import org.firstinspires.ftc.teamcode.hardware.robots.Prometheus;

@TeleOp(name = "Example TeleOp", group = "")
public class ExampleTeleop extends BaseTeleOp {

    private Prometheus prometheus = null;
    private PrometheusConfigTwo gamepadConfig = null;

    @Override
    public void initializeClass() {
        prometheus = new Prometheus(this);
        gamepadConfig = new PrometheusConfigTwo(gamepad1, gamepad2);

        setRobot(prometheus);
        setGamepadConfig(gamepadConfig);
    }

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        //Do this in base class to robot variable
        prometheus.handleOperations(gamepadConfig.getStates());
    }
}
